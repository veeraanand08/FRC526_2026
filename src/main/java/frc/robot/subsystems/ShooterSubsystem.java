package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.DriverConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax motorA;
    private final SparkMax motorB;

    private final SparkMaxConfig configA;
    private final SparkMaxConfig configB;

    public ShooterSubsystem() {
        motorA = new SparkMax(DriverConstants.kShooterMotorA, MotorType.kBrushless);
        motorB = new SparkMax(DriverConstants.kShooterMotorB, MotorType.kBrushless);
        configA = new SparkMaxConfig();
        configB = new SparkMaxConfig();

        configA.inverted(DriverConstants.kShooterMotorAReversed);
        configB.inverted(DriverConstants.kShooterMotorBReversed);

        configA.idleMode(IdleMode.kBrake);
        configB.idleMode(IdleMode.kBrake);

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
