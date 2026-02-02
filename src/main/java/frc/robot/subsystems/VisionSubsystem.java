// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.getSwerveDrive();
    this.gyro = (AHRS) swerveDrive.getGyro().getIMU();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command autoAlign() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    Pose2d robotPose2d = swerveSubsystem.getPose();
    Pose2d targetPose2d = getTargetPose();

    



    
    return swerveSubsystem.driveFieldOriented(() -> swerveSubsystem.rotateToAngle( 
          m_driverController.getLeftY(),
          m_driverController.getLeftX(),
          swerveSubsystem.getHeading().plus(Rotation2d.fromDegrees(visionSubsystem.getLimelightAngle())),
          DriverConstants.DEADBAND
        ));
  }

  

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    updateOrientation();
    updateVision();
  }

  public Pose2d getTargetPose() { //in the future make this also do shoot on the fly offsets
    if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
      return FieldConstants.RED_HUB;
    return FieldConstants.BLUE_HUB;
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
