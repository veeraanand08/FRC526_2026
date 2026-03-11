package frc.robot.subsystems.intake;

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

public class IntakeIOSparkMax implements IntakeIO {
    private final SparkMax pivot;
    private final SparkMax roller;
    private final SparkMaxConfig pivotConfig;

    private final RelativeEncoder pivotEncoder;
    private final RelativeEncoder rollerEncoder;
    private final SparkClosedLoopController pivotPid;
    private final SparkClosedLoopController rollerPid;

    public IntakeIOSparkMax() {
        pivot = new SparkMax(Constants.IntakeConstants.PIVOT_MOTOR, SparkLowLevel.MotorType.kBrushless);
        roller = new SparkMax(Constants.IntakeConstants.ROLLER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        pivotConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        pivotConfig.inverted(Constants.IntakeConstants.PIVOT_REVERSED);
        pivotConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        pivotConfig.smartCurrentLimit(Constants.IntakeConstants.PIVOT_CURRENT_LIMIT);
        pivotConfig.encoder.positionConversionFactor(Constants.IntakeConstants.PIVOT_ROT_TO_DEG);
        pivotConfig.closedLoop.pid(Constants.IntakeConstants.PIVOT_P, Constants.IntakeConstants.PIVOT_I, Constants.IntakeConstants.PIVOT_D)
                .feedForward/*.kS(IntakeConstants.PIVOT_FF_S)*/
                            .kV(Constants.IntakeConstants.PIVOT_FF_V);
                            // .kCos(IntakeConstants.PIVOT_FF_COS)
                            // .kCosRatio(IntakeConstants.PIVOT_FF_COS_RATIO);
        pivotConfig.closedLoop.maxMotion.cruiseVelocity(Constants.IntakeConstants.PIVOT_CRUISE_VELOCITY)
                .maxAcceleration(Constants.IntakeConstants.PIVOT_MAX_ACCEL)
                .allowedProfileError(Constants.IntakeConstants.ALLOWED_PROFILE_ERROR);

        rollerConfig.inverted(Constants.IntakeConstants.ROLLER_REVERSED);
        rollerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(Constants.IntakeConstants.ROLLER_CURRENT_LIMIT);
        rollerConfig.closedLoop.pid(Constants.IntakeConstants.ROLLER_P, Constants.IntakeConstants.ROLLER_I, Constants.IntakeConstants.ROLLER_D)
                .feedForward.kV(Constants.IntakeConstants.ROLLER_FF);

        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotEncoder = pivot.getEncoder();
        pivotEncoder.setPosition(0);
        rollerEncoder = roller.getEncoder();
        pivotPid = pivot.getClosedLoopController();
        rollerPid = roller.getClosedLoopController();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotConnected = !pivot.hasActiveFault();
        inputs.pivotAppliedVolts = pivot.getBusVoltage() * pivot.getAppliedOutput();
        inputs.pivotCurrentAmps = pivot.getOutputCurrent();
        inputs.pivotPositionDeg = pivotEncoder.getPosition();
        inputs.pivotVelocityDegPerSec = pivotEncoder.getVelocity();

        inputs.rollerConnected = !roller.hasActiveFault();
        inputs.rollerAppliedVolts =  roller.getBusVoltage() * roller.getAppliedOutput();
        inputs.rollerCurrentAmps = roller.getOutputCurrent();
        inputs.rollerCurrentRPM = rollerEncoder.getVelocity();
    }

    @Override
    public void setPivotBrake(boolean brake) {
        pivotConfig.idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        pivot.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setPivot(double speed) {
        pivot.set(speed);
    }

    @Override
    public void setRoller(double speed) {
        roller.set(speed);
    }

    @Override
    public void setPivotDeg(double deg) {
        pivotPid.setSetpoint(deg, SparkBase.ControlType.kPosition);
    }

    @Override
    public void setRollerRPM(double rpm) {
        rollerPid.setSetpoint(rpm, SparkBase.ControlType.kVelocity);
    }

    @Override
    public void stopPivot() {
        pivot.stopMotor();
    }

    @Override
    public void stopRoller() {
        roller.stopMotor();
    }
}
