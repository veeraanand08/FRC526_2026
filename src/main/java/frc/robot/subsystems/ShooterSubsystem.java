package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotUtil;
import frc.robot.commands.AutoAlign;

import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax leftMotor; // leader
  private final SparkMax rightMotor; // follower

  private final RelativeEncoder leftMotorEncoder;
  private final RelativeEncoder rightMotorEncoder;
  private final SparkClosedLoopController leftMotorPid;
  private final SlewRateLimiter limit;

  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotVelocity;

  private double leftActualRPM;
  private double rightActualRPM;
  private double desiredRPM;
  private double distanceToTarget;
  private boolean isShooterReady;

  public ShooterSubsystem(Supplier<Pose2d> robotPose,  Supplier<ChassisSpeeds> robotVelocity) {
    leftMotor = new SparkMax(ShooterConstants.LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
    rightMotor = new SparkMax(ShooterConstants.RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);
    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

    leftMotorConfig.inverted(ShooterConstants.MOTORS_REVERSED);
    leftMotorConfig.idleMode(IdleMode.kCoast);
    leftMotorConfig.smartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    leftMotorConfig.voltageCompensation(12.0);
    leftMotorConfig.closedLoop.pid(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D)
                              .feedForward.kV(ShooterConstants.SHOOTER_FF);

    rightMotorConfig.idleMode(IdleMode.kCoast);
    rightMotorConfig.smartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    rightMotorConfig.voltageCompensation(12.0);
    rightMotorConfig.follow(ShooterConstants.LEFT_SHOOTER_MOTOR, true);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftMotorEncoder = leftMotor.getEncoder();
    rightMotorEncoder = rightMotor.getEncoder();
    leftMotorPid = leftMotor.getClosedLoopController();
    limit = new SlewRateLimiter(5676, -ShooterConstants.NEGATIVE_RATE_LIMIT, 0);

    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;

    SmartDashboard.setDefaultNumber("Shooter/Desired Shooter RPM", 0);
    SmartDashboard.setDefaultNumber("Shooter/Tuning RPM", 3000);
    SmartDashboard.putBoolean("Shooter/Tuning Mode Active", ShooterConstants.TUNING_MODE_ACTIVE);
  }

  @Override
  public void periodic() {
    Translation2d robotTranslation = robotPose.get().getTranslation();
    Translation2d virtualTarget;
    if (AutoAlign.isActive()) virtualTarget = AutoAlign.getSavedVirtualTarget();
    else virtualTarget = AutoAlign.getVirtualTarget(robotVelocity.get(), robotTranslation,
                                                    AutoAlign.getTargetTranslation(AutoAlign.Target.HUB, robotTranslation));
    distanceToTarget = robotTranslation.getDistance(virtualTarget);
    isShooterReady = RobotUtil.isPoseEstimatorReady && AutoAlign.isActive();
    leftActualRPM = leftMotorEncoder.getVelocity();
    rightActualRPM = rightMotorEncoder.getVelocity();
    SmartDashboard.putNumber("Shooter/Left Motor RPM", leftActualRPM);
    SmartDashboard.putNumber("Shooter/Right Motor RPM", rightActualRPM);
    SmartDashboard.putBoolean("Shooter/Shooter Ready", isShooterReady);
    SmartDashboard.putString("AutoAlign/Current Target", AutoAlign.getCurrentTarget().toString());
    SmartDashboard.putNumber("Shooter/Distance to Target", distanceToTarget);
  }

  public void setAngularVelocity(double rpm) {
    rpm = limit.calculate(rpm);
    leftMotorPid.setSetpoint(rpm, ControlType.kVelocity);
    desiredRPM = rpm;
    SmartDashboard.putNumber("Shooter/Desired Shooter RPM", desiredRPM);
  }

  public void shoot() {
    if (ShooterConstants.TUNING_MODE_ACTIVE) {
      setAngularVelocity(SmartDashboard.getNumber("Shooter/Tuning RPM", 0));
    }
    else {
      setAngularVelocity(ShooterConstants.DISTANCE_TO_RPM.get(distanceToTarget));
    }
  }

  public boolean hasSpunUp() {
    return leftActualRPM > desiredRPM-100;
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
    desiredRPM = 0;
    SmartDashboard.putNumber("Shooter/Desired Shooter RPM", desiredRPM);
  }
}
