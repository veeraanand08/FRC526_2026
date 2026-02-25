package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
  /* The current state of the pivot motor. */
  public enum PivotState {
    RAISED,
    RAISING,
    AGITATING,
    LOWERING,
    LOWERED
  }

  public PivotState pivotState = PivotState.RAISED;

  private final SparkMax pivotMotor;
  private final SparkMax rollerMotor;
  private final SparkMaxConfig pivotConfig;
  private final SparkMaxConfig rollerConfig;

  private final RelativeEncoder pivotEncoder;
  private final SparkClosedLoopController pivotPid;
  private final SparkClosedLoopController rollerPid;

  private double currentPivotDeg;
  private boolean rollerEnabled;

  public IntakeSubsystem() {
    pivotMotor = new SparkMax(IntakeConstants.PIVOT_MOTOR, MotorType.kBrushless);
    rollerMotor = new SparkMax(IntakeConstants.ROLLER_MOTOR, MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();
    rollerConfig = new SparkMaxConfig();

    pivotConfig.inverted(IntakeConstants.PIVOT_REVERSED);
    pivotConfig.idleMode(IdleMode.kCoast);
    pivotConfig.smartCurrentLimit(IntakeConstants.PIVOT_CURRENT_LIMIT);
    pivotConfig.encoder.positionConversionFactor(IntakeConstants.PIVOT_ROT_TO_DEG);
    pivotConfig.closedLoop.pid(IntakeConstants.PIVOT_P, IntakeConstants.PIVOT_I, IntakeConstants.PIVOT_D)
                          .feedForward/*.kS(IntakeConstants.PIVOT_FF_S)*/
                                      .kV(IntakeConstants.PIVOT_FF_V);
                                      // .kCos(IntakeConstants.PIVOT_FF_COS)
                                      // .kCosRatio(IntakeConstants.PIVOT_FF_COS_RATIO);
    pivotConfig.closedLoop.maxMotion.cruiseVelocity(IntakeConstants.PIVOT_CRUISE_VELOCITY)
                                    .maxAcceleration(IntakeConstants.PIVOT_MAX_ACCEL)
                                    .allowedProfileError(IntakeConstants.ALLOWED_PROFILE_ERROR);

    rollerConfig.inverted(IntakeConstants.ROLLER_REVERSED);
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.smartCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT);
    rollerConfig.closedLoop.pid(IntakeConstants.ROLLER_P, IntakeConstants.ROLLER_I, IntakeConstants.ROLLER_D)
                           .feedForward.kV(IntakeConstants.ROLLER_FF);

    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPosition(0);
    pivotPid = pivotMotor.getClosedLoopController();
    rollerPid = rollerMotor.getClosedLoopController();

    SmartDashboard.setDefaultString("Intake/Pivot State", pivotState.toString());
    SmartDashboard.setDefaultBoolean("Intake/Intake Running", false);
    SmartDashboard.setDefaultBoolean("Intake/Intake Reversed", false);
  }

  @Override
  /* Periodically raises/lowers the pivot depending on its current state. Will not run if in lowered/lowering state. */
  public void periodic() {
    currentPivotDeg = getPivotDeg();
    SmartDashboard.putNumber("Intake/Pivot Degrees", currentPivotDeg);
    switch (pivotState) {
      case RAISING:
        setPivotAngle(IntakeConstants.PIVOT_RAISED_ANGLE);
        if (currentPivotDeg <= IntakeConstants.PIVOT_RAISED_ANGLE) {
          pivotState = PivotState.RAISED;
        }
        break;
      case AGITATING:
        double pos = Math.sin(System.currentTimeMillis() * Math.PI / IntakeConstants.AGITATION_PERIOD) * 0.5 + 0.5;
        double targetAngle = IntakeConstants.PIVOT_AGITATION_UPPER_ANGLE + (IntakeConstants.PIVOT_AGITATION_LOWER_ANGLE-IntakeConstants.PIVOT_AGITATION_UPPER_ANGLE) * pos;
        setPivotAngle(targetAngle);
        break;
      case LOWERING:
        setPivotAngle(IntakeConstants.PIVOT_ENGAGED_ANGLE);
        if (currentPivotDeg >= IntakeConstants.PIVOT_ENGAGED_ANGLE-5) {
          pivotState = PivotState.LOWERED;
          setRoller(true);
        }
        break;
      case LOWERED:
        setPivotAngle(IntakeConstants.PIVOT_ENGAGED_ANGLE);
        break;
      default:
    }
    SmartDashboard.putString("Intake/Pivot State", pivotState.toString());
  }

  public void toggleRoller() {
    // if roller motor is inactive, start it
    setRoller(!rollerEnabled);
  }

  public void setRoller(boolean enabled) {
    if (enabled) rollerPid.setSetpoint(IntakeConstants.ROLLER_RPM, ControlType.kVelocity);
    else stopRoller();
    rollerEnabled = enabled;
    SmartDashboard.putBoolean("Intake/Intake Running", rollerEnabled);
  }

  public void slowRoller() {
    rollerMotor.set(IntakeConstants.ROLLER_POWER_SLOW);
  }

  public void setRollerReversed(boolean enabled) {
    if (enabled) rollerPid.setSetpoint(IntakeConstants.ROLLER_RPM_REVERSED, ControlType.kVelocity);
    else rollerMotor.set(0);
    SmartDashboard.putBoolean("Intake/Intake Reversing", enabled);
  }

  /**
   * Set the pivot motor speed according to a given angle. 
   * @param angle : Angle (in degrees) to rotate.
   */
  public void setPivotAngle(double angle) {
    pivotPid.setSetpoint(angle, ControlType.kPosition);
  }

  /* Returns the current pivot angle (in degrees). */
  public double getPivotDeg() {
    return pivotEncoder.getPosition();
  }

  /* Set the pivot and roller motor speeds to 0. */
  public void stop() {
    setRoller(false);
    stopPivot();
  }

  /* Set the pivot motor speed to 0. */
  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  /* Set the roller motor speed to 0. */
  public void stopRoller() {
    rollerMotor.stopMotor();
  }

  public void setPivotBrake(boolean brake) {
    pivotConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Toggle the intake roller. If the intake is raised, it will lower and then start.
   *
   * @return a command to toggle the intake
   */
  public Command toggleIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          switch (pivotState) {
            case LOWERING:
            case LOWERED:
              toggleRoller();
              break;
            default:
              setRoller(true);
              pivotState = PivotState.LOWERING;
              break;
          }
        });
  }
  
  /**
   * Reverse the intake roller while held
   *
   * @return a command to reverse the intake
   */
  public Command reverseIntakeCommand() {
    // Inline construction of command goes here.
    return startEnd(
      () -> setRollerReversed(true), 
      () -> setRollerReversed(false)
    );
  }

  /**
   * Toggle intake agitation
   *
   * @return a command to agitate the intake
   */
  public Command agitateCommand() {
    return runOnce(() -> {
      if (pivotState == PivotState.AGITATING) {
        setRoller(true);
        pivotState = PivotState.LOWERING;
      }
      else {
        pivotState = PivotState.AGITATING;
        slowRoller();
      }
    });
  }

  /**
   * Stop the roller motor and bring the intake back up
   *
   * @return a command to reset the intake back to starting position
   */
  public Command resetIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setRoller(false);
          pivotState = PivotState.RAISING;
        });
  }

  @Override
  public void simulationPeriodic() {}
}
