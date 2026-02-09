package frc.robot.subsystems;
import org.dyn4j.geometry.Rotation;

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
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
  /* The current state of the pivot motor. */
  public enum PivotState {
    RAISED,
    RAISING,
    AGITATING_UP { // raising during agitation
      @Override
      public String toString() {
        return "AGITATING";
      }
    },
    AGITATING_DOWN { // lowering during agitation
      @Override
      public String toString() {
        return "AGITATING";
      }
    },
    LOWERING,
    LOWERED
  }

  public PivotState pivotState = PivotState.RAISED;

  private final SparkMax pivotMotor;
  private final SparkMax rollerMotor;
  private final SparkMaxConfig pivotConfig;
  private final SparkMaxConfig rollerConfig;

  private final RelativeEncoder pivotEncoder;
  private final SparkClosedLoopController pivotPIDController;

  private double currentPivotDeg;

  public IntakeSubsystem() {
    pivotMotor = new SparkMax(IntakeConstants.PIVOT_MOTOR, MotorType.kBrushless);
    rollerMotor = new SparkMax(IntakeConstants.ROLLER_MOTOR, MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();
    rollerConfig = new SparkMaxConfig();

    pivotConfig.idleMode(IdleMode.kBrake);
    rollerConfig.idleMode(IdleMode.kCoast);
    pivotConfig.closedLoop.pid(IntakeConstants.PIVOT_P, IntakeConstants.PIVOT_I, IntakeConstants.PIVOT_D);
    pivotConfig.encoder.positionConversionFactor(IntakeConstants.PIVOT_ROT_TO_DEG);

    pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rollerMotor.configure(rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotEncoder = pivotMotor.getEncoder();
    pivotPIDController = pivotMotor.getClosedLoopController();

    SmartDashboard.putString("Intake/Pivot State", pivotState.toString());
  }

  @Override
  /* Periodically raises/lowers the pivot depending on its current state. Will not run if in lowered/lowering state. */
  public void periodic() {
    currentPivotDeg = getPivotDeg();
    SmartDashboard.putNumber("Intake/Pivot Degrees", currentPivotDeg);
    switch (pivotState) {
      case RAISING:
        setPivotAngle(IntakeConstants.INTAKE_RAISED_ANGLE);
        if (currentPivotDeg <= IntakeConstants.INTAKE_RAISED_ANGLE) {
          stop();
          pivotState = PivotState.RAISED;
        }
        break;
      case AGITATING_UP:
        setPivotAngle(IntakeConstants.INTAKE_AGITATION_UPPER_ANGLE);
        if (currentPivotDeg <= IntakeConstants.INTAKE_AGITATION_UPPER_ANGLE+5)
          pivotState = PivotState.AGITATING_DOWN;
        break;
      case AGITATING_DOWN:
        setPivotAngle(IntakeConstants.INTAKE_AGITATION_LOWER_ANGLE);
        if (currentPivotDeg >= IntakeConstants.INTAKE_AGITATION_LOWER_ANGLE-5)
          pivotState = PivotState.AGITATING_UP;
        break;
      case LOWERING:
        setPivotAngle(IntakeConstants.INTAKE_ENGAGED_ANGLE);
        if (currentPivotDeg >= IntakeConstants.INTAKE_ENGAGED_ANGLE-5) {
          stopPivot();
          pivotState = PivotState.LOWERED;
          setRoller(true);
        }
        break;
      default:
    }
    SmartDashboard.putString("Intake/Pivot State", pivotState.toString());
  }

  public void toggleRoller() {
    // if roller motor is inactive, start it
    setRoller(rollerMotor.get() == 0);
  }

  public void setRoller(boolean enabled) {
    rollerMotor.set(enabled ? IntakeConstants.ROLLER_POWER : 0);
    SmartDashboard.putBoolean("Intake/Intake Running", enabled);
  }

  /** Set the pivot motor speed according to a given angle. 
   * @param angle : Angle (in degrees) to rotate.
   */
  public void setPivotAngle(double angle) {
    pivotPIDController.setSetpoint(angle, ControlType.kPosition);
  }

  /* Returns the current pivot angle (in degrees). */
  public double getPivotDeg() {
    return pivotEncoder.getPosition();
  }

  @Override
  public void simulationPeriodic() {}

  /**
   * Toggle the intake roller
   *
   * @return a command to toggle the intake
   */
  public Command toggleIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          switch (pivotState) {
            case RAISED:
              pivotState = PivotState.LOWERING;
              break;
            case LOWERED:
              toggleRoller();
              break;
            default:
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
          pivotState = PivotState.RAISING;
        });
  }

  /* Set the pivot and roller motor speeds to 0. */
  public void stop() {
    setRoller(false);
    stopPivot();
  }

  /* Set the pivot motor speed to 0. */
  public void stopPivot() {
    pivotMotor.set(0);
  }
}
