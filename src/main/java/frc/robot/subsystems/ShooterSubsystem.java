package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.commands.AutoAlign;
import swervelib.SwerveDrive;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  private final VisionSubsystem visionSubsystem;

  private final SparkMax leftMotor; // leader
  private final SparkMax rightMotor; // follower
  private final SparkMaxConfig leftMotorConfig;
  private final SparkMaxConfig rightMotorConfig;

  private final RelativeEncoder leftMotorEncoder;
  private final RelativeEncoder rightMotorEncoder;
  private final SparkClosedLoopController leftMotorPid;

  private double leftActualRPM;
  private double rightActualRPM;
  private double distanceToTarget;
  private boolean isShooterReady;

  public ShooterSubsystem(SwerveDrive swerveDrive, VisionSubsystem visionSubsystem) {
    this.swerveDrive = swerveDrive;
    this.visionSubsystem = visionSubsystem;

    leftMotor = new SparkMax(ShooterConstants.LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
    rightMotor = new SparkMax(ShooterConstants.RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);
    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();

    leftMotorConfig.inverted(ShooterConstants.MOTORS_REVERSED);
    leftMotorConfig.idleMode(IdleMode.kCoast);
    rightMotorConfig.idleMode(IdleMode.kCoast);
    leftMotorConfig.closedLoop.pid(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D)
                              .feedForward.kV(ShooterConstants.SHOOTER_FF);
    rightMotorConfig.follow(ShooterConstants.LEFT_SHOOTER_MOTOR, true);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftMotorEncoder = leftMotor.getEncoder();
    rightMotorEncoder = rightMotor.getEncoder();
    leftMotorPid = leftMotor.getClosedLoopController();

    SmartDashboard.setDefaultNumber("Shooter/Desired Shooter RPM", 0);
    SmartDashboard.setDefaultNumber("Shooter/Tuning RPM", 0);
    SmartDashboard.putBoolean("Shooter/Tuning Mode Active", ShooterConstants.TUNING_MODE_ACTIVE);
  }

  @Override
  public void periodic() {
    Translation2d robotTranslation = swerveDrive.getPose().getTranslation();
    distanceToTarget = robotTranslation.getDistance(AutoAlign.getTargetTranslation(AutoAlign.getCurrentTarget(), robotTranslation));
    isShooterReady = visionSubsystem.isPoseEstimatorReady() && distanceToTarget <= ShooterConstants.MAX_DISTANCE;
    leftActualRPM = leftMotorEncoder.getVelocity();
    rightActualRPM = rightMotorEncoder.getVelocity();
    SmartDashboard.putNumber("Shooter/Left Motor RPM", leftActualRPM);
    SmartDashboard.putNumber("Shooter/Right Motor RPM", rightActualRPM);
    SmartDashboard.putBoolean("Shooter/Shooter Ready", isShooterReady);
    SmartDashboard.putString("AutoAlign/Current Target", AutoAlign.getCurrentTarget().toString());
    SmartDashboard.putNumber("Shooter/Distance to Target", distanceToTarget);
  }

  public void setAngularVelocity(double rpm) {
    leftMotorPid.setSetpoint(rpm, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter/Desired Shooter RPM", rpm);
  }

  public void shoot() {
    if (ShooterConstants.TUNING_MODE_ACTIVE) {
      setAngularVelocity(SmartDashboard.getNumber("Shooter/Tuning RPM", 0));
    }
    else {
      setAngularVelocity(ShooterConstants.DISTANCE_TO_RPM.get(distanceToTarget));
    }
  }

  public void stop() {
    setAngularVelocity(0);
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
