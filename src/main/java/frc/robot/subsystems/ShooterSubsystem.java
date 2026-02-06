package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoAlign;
import swervelib.SwerveDrive;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;

  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final SparkMaxConfig leftMotorConfig;
  private final SparkMaxConfig rightMotorConfig;

  private final RelativeEncoder leftMotorEncoder;
  private final RelativeEncoder rightMotorEncoder;
  private final PIDController leftMotorPid;
  private final PIDController rightMotorPid;

  private boolean isShooterReady;
  
  public ShooterSubsystem(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;

    leftMotor = new SparkMax(ShooterConstants.LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
    rightMotor = new SparkMax(ShooterConstants.RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);
    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();

    leftMotorConfig.inverted(ShooterConstants.LEFT_SHOOTER_MOTOR_REVERSED);
    rightMotorConfig.inverted(ShooterConstants.RIGHT_SHOOTER_MOTOR_REVERSED);
    leftMotorConfig.idleMode(IdleMode.kBrake);
    rightMotorConfig.idleMode(IdleMode.kBrake);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftMotorEncoder = leftMotor.getEncoder();
    rightMotorEncoder = rightMotor.getEncoder();
    leftMotorPid = new PIDController(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D);
    rightMotorPid = new PIDController(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D);
  }

  public void setAngularVelocity(double rpm) {
    leftMotor.set(leftMotorPid.calculate(leftMotorEncoder.getVelocity(), rpm));
    rightMotor.set(rightMotorPid.calculate(rightMotorEncoder.getVelocity(), rpm));
  }

  public void shoot() {
    Translation2d robotTranslation = swerveDrive.getPose().getTranslation();
    double distanceToTarget = robotTranslation.getDistance(AutoAlign.getTargetTranslation(AutoAlign.getCurrentTarget(), robotTranslation));
    setAngularVelocity(ShooterConstants.DISTANCE_TO_RPM.get(distanceToTarget));
  }
}
