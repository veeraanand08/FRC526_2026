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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.controller.PIDController;

public class IntakeSubsystem extends SubsystemBase {
  /* The current state of the pivot motor. */
  public enum PivotState {
    RAISED,
    SHAKING_UP, // raising during shaking
    SHAKING_DOWN, // lowering during shaking
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

    pivotPIDController = pivotMotor.getClosedLoopController();

    pivotEncoder = pivotMotor.getEncoder();
  }

  @Override
  /* Periodically raises/lowers the pivot depending on its current state. Will not run if in lowered/lowering state. */
  public void periodic() {
    double currentDeg = getPivotDeg();
    switch (pivotState) {
      case SHAKING_UP:
        setPivotAngle(IntakeConstants.INTAKE_AGITATION_UPPER_ANGLE);
        if (currentDeg <= IntakeConstants.INTAKE_AGITATION_UPPER_ANGLE+5)
          pivotState = PivotState.SHAKING_DOWN;
        break;
      case SHAKING_DOWN:
        setPivotAngle(IntakeConstants.INTAKE_AGITATION_LOWER_ANGLE);
        if (currentDeg >= IntakeConstants.INTAKE_AGITATION_LOWER_ANGLE-5)
          pivotState = PivotState.SHAKING_UP;
        break;
      case LOWERING:
        setPivotAngle(IntakeConstants.INTAKE_ENGAGED_ANGLE);
        if (currentDeg >= IntakeConstants.INTAKE_ENGAGED_ANGLE-5) {
          pivotState = PivotState.LOWERED;
          stopPivot();
          setRoller(true);
        }
        break;
      default:
    }
  }

  /* Turns on the roller motor if off, and vice-versa */
  public void setRoller(boolean enabled) {
    rollerMotor.set(enabled ? IntakeConstants.ROLLER_POWER : 0);
  }

  /** Set the pivot motor speed according to a given angle. 
   * @param angle : Angle (in degrees) to rotate.
   */
  public void setPivotAngle(double angle) {
    pivotPIDController.setSetpoint(angle, ControlType.kPosition);
  }

  /* Returns the current pivot angle (in degrees). */
  public double getPivotDeg() {
    return Math.toDegrees(pivotEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {}

  /* Set the pivot and roller motor speeds to 0. */
  public void stop() {
    rollerMotor.set(0);
    stopPivot();
  }

  /* Set the pivot motor speed to 0. */
  public void stopPivot() {
    pivotMotor.set(0);
  }
}
