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

  public boolean isPoseEstimatorReady() {
    return isPoseEstimatorReady;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    swerveDrive.updateOdometry();
    updateOrientation();
    updateOdometry();
  }
}