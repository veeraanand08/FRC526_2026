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
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final SparkMaxConfig leftConfig;
    private final SparkMaxConfig rightConfig;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        leftMotor = new SparkMax(DriverConstants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
        rightMotor = new SparkMax(DriverConstants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);
        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public void setMotorSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
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
}
