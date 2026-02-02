// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.FieldConstants;
import limelight.Limelight;
import limelight.networktables.*;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import swervelib.SwerveDrive;

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

  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.getSwerveDrive();
    this.gyro = (AHRS) swerveDrive.getGyro().getIMU();
  }

  public Command autoAlign(CommandXboxController m_driverController) {
    return run(() -> {
      Pose2d robotPose2d = swerveSubsystem.getPose();
      Pose2d targetPose2d = getTargetPose();

      Translation2d diffrence = targetPose2d.getTranslation().minus(robotPose2d.getTranslation());
      
      Rotation2d angleToTarget = new Rotation2d(diffrence.getX(), diffrence.getY());
    
      swerveSubsystem.driveFieldOriented(swerveSubsystem.rotateToAngle( 
          m_driverController.getLeftY(),
          m_driverController.getLeftX(),
          angleToTarget,
          DriverConstants.DEADBAND
      ));
    });
  }

  private Pose2d getTargetPose() { //in the future make this also do shoot on the fly offsets
    if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
      return FieldConstants.RED_HUB;
    return FieldConstants.BLUE_HUB;
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    updateOrientation();
    updateVision();
  }

  private void updateOrientation() {
    Orientation3d orientation = new Orientation3d(gyro.getRotation3d(),
        new AngularVelocity3d(DegreesPerSecond.of(gyro.getRawGyroX()),
            DegreesPerSecond.of(gyro.getRawGyroY()),
            DegreesPerSecond.of(gyro.getRawGyroZ())));
    
    limelightLeft.getSettings().withRobotOrientation(orientation).save();
    limelightRight.getSettings().withRobotOrientation(orientation).save();
  }

  private void updateVision() {
    double currentTime = Timer.getFPGATimestamp();
    limelightLeftPoseEstimator.getPoseEstimate().ifPresent((PoseEstimate poseEstimate) -> {
        if (poseEstimate.tagCount > 0 &&
            (currentTime - poseEstimate.timestampSeconds) < 0.1)
        {
          swerveDrive.addVisionMeasurement(
            poseEstimate.pose.toPose2d(), 
            poseEstimate.timestampSeconds
          );
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
        }
    });
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
