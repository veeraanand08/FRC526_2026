package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ModuleConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax motorA;
    private final SparkMax motorB;

    private final SparkMaxConfig configA;
    private final SparkMaxConfig configB;

    private final RelativeEncoder encoderA;
    private final RelativeEncoder encoderB;

    private final PIDController PIDA;
    private final PIDController PIDB;

    public ShooterSubsystem() {
        motorA = new SparkMax(DriverConstants.kShooterMotorA, MotorType.kBrushless);
        motorB = new SparkMax(DriverConstants.kShooterMotorB, MotorType.kBrushless);
        configA = new SparkMaxConfig();
        configB = new SparkMaxConfig();

        configA.inverted(DriverConstants.kShooterMotorAReversed);
        configB.inverted(DriverConstants.kShooterMotorBReversed);

        configA.idleMode(IdleMode.kBrake);
        configA.encoder.positionConversionFactor(ModuleConstants.kShooterEncoderRot2Rad);
        configB.idleMode(IdleMode.kBrake);
        configB.encoder.positionConversionFactor(ModuleConstants.kShooterEncoderRot2Rad);

        motorA.configure(configA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorB.configure(configB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoderA = motorA.getEncoder();
        encoderB = motorB.getEncoder();

        PIDA = new PIDController(ModuleConstants.kPShooter, ModuleConstants.kIShooter, ModuleConstants.kDShooter);
        PIDB = new PIDController(ModuleConstants.kPShooter, ModuleConstants.kIShooter, ModuleConstants.kDShooter);
    }

    public void start(double speed) {
      motorA.set(PIDA.calculate(encoderA.getVelocity(), speed));
      motorB.set(PIDB.calculate(encoderB.getVelocity(), speed));
    }

    public void stop() {
      motorA.set(0);
      motorB.set(0);
    }
}
