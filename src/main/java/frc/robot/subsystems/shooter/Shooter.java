package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotUtil;
import frc.robot.commands.AutoAlign;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final SlewRateLimiter limit;

  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotVelocity;

  private double desiredRPM;
  private double distanceToTarget;

  public Shooter(ShooterIO io, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
    this.io = io;

    limit = new SlewRateLimiter(5676, -ShooterConstants.NEGATIVE_RATE_LIMIT, 0);

    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;

    Logger.recordOutput("Shooter/Shooter Ready", false);
    Logger.recordOutput("Shooter/Desired Shooter RPM", 0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Translation2d robotTranslation = robotPose.get().getTranslation();
    Translation2d virtualTarget;
    boolean autoAlignActive = AutoAlign.isActive();

    if (autoAlignActive) virtualTarget = AutoAlign.getSavedVirtualTarget();
    else virtualTarget = AutoAlign.getVirtualTarget(robotVelocity.get(), robotTranslation,
                                                    AutoAlign.getTargetTranslation(AutoAlign.Target.HUB, robotTranslation));
    distanceToTarget = robotTranslation.getDistance(virtualTarget);

    Logger.recordOutput("Shooter/Shooter Ready", RobotUtil.shiftTimer.isHubActive() && autoAlignActive);
    Logger.recordOutput("Shooter/Distance to Target", distanceToTarget);
  }

  public void shoot() {
    setAngularVelocity(ShooterConstants.DISTANCE_TO_RPM.get(distanceToTarget));
  }

  public void setAngularVelocity(double rpm) {
    rpm = limit.calculate(rpm);
    io.setRPM(rpm);
    desiredRPM = rpm;
    Logger.recordOutput("Shooter/Desired Shooter RPM", desiredRPM);
  }

  public boolean hasSpunUp() {
    return inputs.leaderCurrentRPM > desiredRPM - 100;
  }

  public void stop() {
    io.stop();
    desiredRPM = 0;
    Logger.recordOutput("Shooter/Desired Shooter RPM", desiredRPM);
  }
}
