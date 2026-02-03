// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import limelight.Limelight;
import limelight.networktables.*;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import swervelib.SwerveDrive;

import java.lang.reflect.Field;
import java.util.Optional;

import com.studica.frc.AHRS;

import static edu.wpi.first.units.Units.*;

public class VisionSubsystem extends SubsystemBase {
  private final Limelight limelightLeft = new Limelight("limelight-left");
  private final Limelight limelightRight = new Limelight("limelight-right");

  private final LimelightPoseEstimator limelightLeftPoseEstimator = limelightLeft.createPoseEstimator(EstimationMode.MEGATAG2);
  private final LimelightPoseEstimator limelightRightPoseEstimator = limelightRight.createPoseEstimator(EstimationMode.MEGATAG2);

  private final SwerveSubsystem swerveSubsystem;
  private final SwerveDrive swerveDrive;
  private final AHRS gyro;

  private boolean isPoseEstimatorReady;

  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.getSwerveDrive();
    this.gyro = (AHRS) swerveDrive.getGyro().getIMU();
  }

  public Command autoAlign(CommandXboxController driverController) {
    // ensure at least one vision measurement has been added
    if (!isPoseEstimatorReady)
      return Commands.none();
    return run(() -> {
      Pose2d robotPose = swerveSubsystem.getPose();
      Pose2d targetPose = getTargetPose(robotPose);

      Translation2d diffrence = targetPose.getTranslation().minus(robotPose.getTranslation());
      Rotation2d angleToTarget = new Rotation2d(diffrence.getX(), diffrence.getY());

      double leftY = (swerveSubsystem.isRedAlliance()) ? driverController.getLeftY() : -driverController.getLeftY();
      double leftX = (swerveSubsystem.isRedAlliance()) ? driverController.getLeftX() : -driverController.getLeftX();

      swerveSubsystem.driveFieldOriented(swerveSubsystem.rotateToAngle( 
          leftY,
          leftX,
          angleToTarget
      ));
    });
  }

  private Pose2d getTargetPose(Pose2d robotPose) {
    Pose2d hubPose;
    // alliance relative
    hubPose = (swerveSubsystem.isRedAlliance()) ? FieldConstants.RED_HUB : FieldConstants.BLUE_HUB;
    
    Translation2d realHubTranslation = hubPose.getTranslation();
    Translation2d robotTranslation = robotPose.getTranslation();
    ChassisSpeeds robotSpeed = swerveDrive.getFieldVelocity();
    Translation2d virtualTargetTranslation = realHubTranslation;

    for (int i = 0; i < ShooterConstants.MAX_ITERATIONS; i++){
      double distanceToTarget = robotTranslation.getDistance(virtualTargetTranslation);
      double shotTime = ShooterConstants.DISTANCE_TO_TIME.get(distanceToTarget);

      double xTranslation = robotSpeed.vxMetersPerSecond * shotTime;
      double yTranslation = robotSpeed.vyMetersPerSecond * shotTime;

      virtualTargetTranslation = realHubTranslation.minus(new Translation2d(xTranslation, yTranslation));
    }

    return new Pose2d(virtualTargetTranslation, Rotation2d.kZero);
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    updateOrientation();
    updateOdometry();
  }

  private void updateOrientation() {
    Orientation3d orientation = new Orientation3d(gyro.getRotation3d(),
        new AngularVelocity3d(DegreesPerSecond.of(gyro.getRawGyroX()),
            DegreesPerSecond.of(gyro.getRawGyroY()),
            DegreesPerSecond.of(gyro.getRawGyroZ())));
    
    limelightLeft.getSettings().withRobotOrientation(orientation).save();
    limelightRight.getSettings().withRobotOrientation(orientation).save();
  }

  private void updateOdometry() {
    double currentTime = Timer.getFPGATimestamp();
    limelightLeftPoseEstimator.getPoseEstimate().ifPresent((PoseEstimate poseEstimate) -> {
        if (poseEstimate.tagCount > 0 &&
            (currentTime - poseEstimate.timestampSeconds) < 0.1)
        {
          swerveDrive.addVisionMeasurement(
            poseEstimate.pose.toPose2d(), 
            poseEstimate.timestampSeconds
          );
          isPoseEstimatorReady = true;
        }
    });
    limelightRightPoseEstimator.getPoseEstimate().ifPresent((PoseEstimate poseEstimate) -> {
        if (poseEstimate.tagCount > 0 &&
            (currentTime - poseEstimate.timestampSeconds) < 0.1)
        {
          swerveDrive.addVisionMeasurement(
            poseEstimate.pose.toPose2d(), 
            poseEstimate.timestampSeconds
          );
          isPoseEstimatorReady = true;
        }
    });
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    swerveDrive.updateOdometry();
    updateOrientation();
    updateOdometry();
  }
}