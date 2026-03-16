package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class FeederIOSparkMax implements FeederIO {
    private final SparkMax indexerLeft;
    private final SparkMax indexerRight;
    private final SparkMax kicker;

    private final RelativeEncoder kickerEncoder;
    private final SparkClosedLoopController kickerPid;

    public FeederIOSparkMax() {
        indexerLeft = new SparkMax(Constants.FeederConstants.LEFT_INDEXER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        indexerRight = new SparkMax(Constants.FeederConstants.RIGHT_INDEXER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        kicker = new SparkMax(Constants.FeederConstants.KICKER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig indexerLeftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig indexerRightMotorConfig = new SparkMaxConfig();
        SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();

        indexerLeftMotorConfig.inverted(Constants.FeederConstants.LEFT_INDEXER_MOTOR_REVERSED);
        indexerLeftMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        indexerLeftMotorConfig.smartCurrentLimit(Constants.FeederConstants.INDEXER_CURRENT_LIMIT);

        indexerRightMotorConfig.inverted(Constants.FeederConstants.RIGHT_INDEXER_MOTOR_REVERSED);
        indexerRightMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        indexerRightMotorConfig.smartCurrentLimit(Constants.FeederConstants.INDEXER_CURRENT_LIMIT);

        kickerMotorConfig.inverted(Constants.FeederConstants.KICKER_MOTOR_REVERSED);
        kickerMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        kickerMotorConfig.smartCurrentLimit(Constants.FeederConstants.KICKER_CURRENT_LIMIT);
        kickerMotorConfig.voltageCompensation(12.0);
        kickerMotorConfig.closedLoop.pid(Constants.FeederConstants.KICKER_P, Constants.FeederConstants.KICKER_I, Constants.FeederConstants.KICKER_D)
                .feedForward.kV(Constants.FeederConstants.KICKER_FF);

        indexerLeft.configure(indexerLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerRight.configure(indexerRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kicker.configure(kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerEncoder = kicker.getEncoder();
        kickerPid = kicker.getClosedLoopController();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.indexerLeftConnected = !indexerLeft.hasActiveFault();
        inputs.indexerLeftAppliedVolts = indexerLeft.getBusVoltage() * indexerLeft.getAppliedOutput();
        inputs.indexerLeftCurrentAmps = indexerLeft.getOutputCurrent();

        inputs.indexerRightConnected = !indexerRight.hasActiveFault();
        inputs.indexerRightAppliedVolts = indexerRight.getBusVoltage() * indexerRight.getAppliedOutput();
        inputs.indexerRightCurrentAmps = indexerRight.getOutputCurrent();

        inputs.kickerConnected = !kicker.hasActiveFault();
        inputs.kickerAppliedVolts = kicker.getBusVoltage() * kicker.getAppliedOutput();
        inputs.kickerCurrentAmps = kicker.getOutputCurrent();
        inputs.kickerCurrentRPM = kickerEncoder.getVelocity();
    }

    @Override
    public void setIndexerLeft(double speed) {
        indexerLeft.set(speed);
    }

    @Override
    public void setIndexerRight(double speed) {
        indexerRight.set(speed);
    }

    @Override
    public void setKicker(double speed) {
        kicker.set(speed);
    }

    @Override
    public void setKickerRPM(double rpm) {
        kickerPid.setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void stopIndexerLeft() {
        indexerLeft.stopMotor();
    }

    @Override
    public void stopIndexerRight() {
        indexerRight.stopMotor();
    }

    @Override
    public void stopKicker() {
        kicker.stopMotor();
    }
}
