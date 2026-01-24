package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax motorA;
    private final SparkMax motorB;

    private final SparkMaxConfig configA;
    private final SparkMaxConfig configB;

  /** Creates a new ExampleSubsystem. */
    public ShooterSubsystem() {
        motorA = new SparkMax(DriverConstants.kShooterMotorA, MotorType.kBrushless);
        motorB = new SparkMax(DriverConstants.kShooterMotorA, MotorType.kBrushless);
        configA = new SparkMaxConfig();
        configB = new SparkMaxConfig();

        configA.inverted(DriverConstants.kShooterMotorAReversed);
        configB.inverted(DriverConstants.kShooterMotorBReversed);

        configA.IdleMode(IdleMode.kBrake);
        configB.IdleMode(IdleMode.kBrake);

        motorA.configure(configA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorB.configure(configB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void start(double speed) {
      motorA.set(speed);
      motorB.set(speed);
    }

    public void stop() {
      motorA.set(0);
      motorB.set(0);
    }
}
