package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax pivotMotor;
    private final SparkMax rollerMotor;

    private final SparkMaxConfig pivotConfig;
    private final SparkMaxConfig rollerConfig;

    private final RelativeEncoder pivotEncoder;
    private final PIDController pivotPIDController;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        pivotMotor = new SparkMax(DriverConstants.PIVOT_INTAKE_MOTOR, MotorType.kBrushless);
        rollerMotor = new SparkMax(DriverConstants.ROLLER_INTAKE_MOTOR, MotorType.kBrushless);
        pivotConfig = new SparkMaxConfig();
        rollerConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kBrake);
        rollerConfig.idleMode(IdleMode.kBrake);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPIDController = new PIDController(ModuleConstants.P_INTAKE, ModuleConstants.I_INTAKE, ModuleConstants.D_INTAKE);
    }

    public void stop() {
        rollerMotor.set(0);
    }

    public void setMotorSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void setPivotPos(double rot) {
        pivotMotor.set(pivotPIDController.calculate(pivotEncoder.getPosition(), rot));
    }

  public Command IntakeMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An Intake method querying a boolean state of the subsystem (for Intake, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean IntakeCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public enum PivotState {
    RAISING, RAISED, LOWERING, LOWERED
  }
}
