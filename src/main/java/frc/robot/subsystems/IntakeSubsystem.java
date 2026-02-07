package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.controller.PIDController;

public class IntakeSubsystem extends SubsystemBase {
    public PivotState pivotState;

    private final SparkMax pivotMotor;
    private final SparkMax rollerMotor;

    private final SparkMaxConfig pivotConfig;
    private final SparkMaxConfig rollerConfig;

    private final RelativeEncoder pivotEncoder;
    private final PIDController pivotPIDController;

    public IntakeSubsystem() {
        pivotMotor = new SparkMax(DriverConstants.PIVOT_INTAKE_MOTOR, MotorType.kBrushless);
        rollerMotor = new SparkMax(DriverConstants.ROLLER_INTAKE_MOTOR, MotorType.kBrushless);
        pivotConfig = new SparkMaxConfig();
        rollerConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kBrake);
        rollerConfig.idleMode(IdleMode.kBrake);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPIDController = new PIDController(ModuleConstants.INTAKE_P, ModuleConstants.INTAKE_I, ModuleConstants.INTAKE_D);
        pivotMotor.configure(
            pivotConfig,
            ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters // According to the docs these parameters are deprecated and I was unable to find a new alternative
        );
      }

    /** Set the pivot and roller motor speeds to 0. */
    public void stop() {
        rollerMotor.set(0);
        pivotMotor.set(0);
    }

    /** Set the pivot motor speed to 0. */
    public void stopPivot() {pivotMotor.set(0);}

    /** Turns on the roller motor if off, and vice-versa */
    public void toggleRoller() {
      rollerMotor.set(rollerMotor.get()==0 ? ModuleConstants.INTAKE_CONSTANT : 0);
    }

    /** Set the pivot motor speed according to a given angle. 
     * @param rot : Rotation (in degrees) to rotate. */
    public void setPivotPos(double rot) {
      pivotMotor.set(pivotPIDController.calculate(pivotEncoder.getPosition(), rot));
    }

    /** Returns the current pivot angle (in degrees). */
    public double getPivotDeg() {
      return Math.toDegrees(pivotEncoder.getPosition());
    }

  @Override
  /* Periodically raises/lowers the pivot depending on its current state. Will not run if in lowered/lowering state. */
  public void periodic() {
    double currentDeg = getPivotDeg();
    if (pivotState==PivotState.RAISED_RAISING) {
      setPivotPos(ModuleConstants.INTAKE_UPPER_RAISED);
      if (currentDeg<=ModuleConstants.INTAKE_UPPER_RAISED+5)
        pivotState = PivotState.RAISED_LOWERING;
    } else if (pivotState==PivotState.RAISED_LOWERING) {
      setPivotPos(ModuleConstants.INTAKE_LOWER_RAISED);
      if (currentDeg>=ModuleConstants.INTAKE_LOWER_RAISED+5)
        pivotState = PivotState.RAISED_RAISING;
    }
  }

  @Override
  public void simulationPeriodic() {}

  /** The current state of the pivot motor. */
  public enum PivotState {RAISED_RAISING, RAISED_LOWERING, LOWERING, LOWERED}
}
