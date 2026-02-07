// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.studica.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import swervelib.SwerveDrive;

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
    this.updateOdometry();
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

  public void toggleLED() {
    // 0 = Pipeline Control, 1 = Force Off, 2 = Force Blink, 3 = Force On
    NetworkTableEntry ledMode = limelightLeft.getNTTable().getEntry("ledMode");
    if (ledMode.equals(1))
      setLED(LEDMode.ForceOn);
    else
      setLED(LEDMode.ForceOff);
  }

  public void setLED(LEDMode ledMode) {
    limelightLeft.getSettings().withLimelightLEDMode(ledMode);
    limelightRight.getSettings().withLimelightLEDMode(ledMode);
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