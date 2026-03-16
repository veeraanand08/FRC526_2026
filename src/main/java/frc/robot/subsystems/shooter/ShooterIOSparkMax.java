package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ShooterIOSparkMax implements ShooterIO {
    private final SparkMax leader;
    private final SparkMax follower;

    private final RelativeEncoder leaderEncoder;
    private final RelativeEncoder followerEncoder;
    private final SparkClosedLoopController shooterPid;

    public ShooterIOSparkMax() {
        leader = new SparkMax(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        follower = new SparkMax(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

        leftMotorConfig.inverted(Constants.ShooterConstants.MOTORS_REVERSED);
        leftMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        leftMotorConfig.smartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
        leftMotorConfig.voltageCompensation(12.0);
        leftMotorConfig.closedLoop.pid(Constants.ShooterConstants.SHOOTER_P, Constants.ShooterConstants.SHOOTER_I, Constants.ShooterConstants.SHOOTER_D)
                .feedForward.kV(Constants.ShooterConstants.SHOOTER_FF);

        rightMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        rightMotorConfig.smartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
        rightMotorConfig.voltageCompensation(12.0);
        rightMotorConfig.follow(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR, true);

        leader.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leaderEncoder = leader.getEncoder();
        followerEncoder = follower.getEncoder();
        shooterPid = leader.getClosedLoopController();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leaderConnected = !leader.hasActiveFault();
        inputs.leaderAppliedVolts = leader.getBusVoltage() * leader.getAppliedOutput();
        inputs.leaderCurrentAmps = leader.getOutputCurrent();
        inputs.leaderCurrentRPM = leaderEncoder.getVelocity();

        inputs.followerConnected = !follower.hasActiveFault();
        inputs.followerAppliedVolts =  follower.getBusVoltage() * follower.getAppliedOutput();
        inputs.followerCurrentAmps = follower.getOutputCurrent();
        inputs.followerCurrentRPM = followerEncoder.getVelocity();
    }

    @Override
    public void set(double speed) {
        leader.set(speed);
    }

    @Override
    public void setRPM(double rpm) {
        shooterPid.setSetpoint(rpm, SparkBase.ControlType.kVelocity);
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }
}
