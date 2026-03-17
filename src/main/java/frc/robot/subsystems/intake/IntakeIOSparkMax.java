package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {
    private final SparkMax pivot;
    private final SparkMax roller;
    private final SparkMaxConfig pivotConfig;

    private final AbsoluteEncoder pivotEncoder;
    private final RelativeEncoder rollerEncoder;
    private final SparkClosedLoopController pivotPid;
    private final SparkClosedLoopController rollerPid;

    public IntakeIOSparkMax() {
        pivot = new SparkMax(IntakeConstants.PIVOT_MOTOR, SparkLowLevel.MotorType.kBrushless);
        roller = new SparkMax(IntakeConstants.ROLLER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        pivotConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        pivotConfig.inverted(IntakeConstants.PIVOT_REVERSED);
        pivotConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        pivotConfig.smartCurrentLimit(IntakeConstants.PIVOT_CURRENT_LIMIT);
        pivotConfig.absoluteEncoder
                .positionConversionFactor(IntakeConstants.PIVOT_ROT_TO_DEG)
                .velocityConversionFactor(IntakeConstants.PIVOT_RPM_TO_DEG_PER_SEC)
                .zeroOffset(IntakeConstants.PIVOT_ENCODER_OFFSET);
        pivotConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(IntakeConstants.PIVOT_P, IntakeConstants.PIVOT_I, IntakeConstants.PIVOT_D);
//                .feedForward.kS(IntakeConstants.PIVOT_FF_S)
//                            .kV(IntakeConstants.PIVOT_FF_V);
//                            .kCos(IntakeConstants.PIVOT_FF_COS)
//                            .kCosRatio(IntakeConstants.PIVOT_FF_COS_RATIO);
        pivotConfig.closedLoop.maxMotion
                .cruiseVelocity(IntakeConstants.PIVOT_CRUISE_VELOCITY)
                .maxAcceleration(IntakeConstants.PIVOT_MAX_ACCEL)
                .allowedProfileError(IntakeConstants.ALLOWED_PROFILE_ERROR);

        rollerConfig.inverted(IntakeConstants.ROLLER_REVERSED);
        rollerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT);
        rollerConfig.voltageCompensation(12.0);
        rollerConfig.closedLoop
                .pid(IntakeConstants.ROLLER_P, IntakeConstants.ROLLER_I, IntakeConstants.ROLLER_D)
                .feedForward.kV(IntakeConstants.ROLLER_FF);

        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotEncoder = pivot.getAbsoluteEncoder();
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
    public void setPivotSetpoint(double deg) {
        pivotPid.setSetpoint(deg, SparkBase.ControlType.kPosition);
    }

    @Override
    public void setPivotDeg(double deg) {
        pivotPid.setSetpoint(deg, SparkBase.ControlType.kMAXMotionPositionControl);
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
