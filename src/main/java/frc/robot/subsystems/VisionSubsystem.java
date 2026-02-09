// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import swervelib.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {
  private final Limelight limelightLeft = new Limelight(VisionConstants.LIMELIGHT_LEFT_NAME);
  private final Limelight limelightRight = new Limelight(VisionConstants.LIMELIGHT_RIGHT_NAME);

  private final LimelightPoseEstimator[] limelightPoseEstimators = {
    limelightLeft.createPoseEstimator(EstimationMode.MEGATAG2),
    limelightRight.createPoseEstimator(EstimationMode.MEGATAG2)
  };

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
    updateOrientation();
    this.updatePose();
  }

  private void updateOrientation() {
    Orientation3d orientation = new Orientation3d(gyro.getRotation3d(),
        new AngularVelocity3d(DegreesPerSecond.of(gyro.getRawGyroX()),
            DegreesPerSecond.of(gyro.getRawGyroY()),
            DegreesPerSecond.of(gyro.getRawGyroZ())));
    
    limelightLeft.getSettings().withRobotOrientation(orientation);
    limelightRight.getSettings().withRobotOrientation(orientation);
  }

  private void updatePose() {
    // account for going over bump
    if (Math.abs(gyro.getRoll()) >= VisionConstants.MAX_TILT_DEG &&
        Math.abs(gyro.getPitch()) >= VisionConstants.MAX_TILT_DEG)
    {
      isPoseEstimatorReady = false;
    }
    for (LimelightPoseEstimator poseEstimator : limelightPoseEstimators) {
      poseEstimator.getPoseEstimate().ifPresent((PoseEstimate poseEstimate) -> {
        if (poseEstimate.tagCount > 0 &&
            (Timer.getTimestamp() - poseEstimate.timestampSeconds) < 0.1 &&
            poseEstimate.getMaxTagAmbiguity() < VisionConstants.MAX_TAG_AMBIGUITY)
        {
          swerveDrive.addVisionMeasurement(
            poseEstimate.pose.toPose2d(), 
            poseEstimate.timestampSeconds
          );
          isPoseEstimatorReady = true;
        }
      });
    }
  }

  public void toggleLED() {
    // 0 = Pipeline Control, 1 = Force Off, 2 = Force Blink, 3 = Force On
    NetworkTableEntry ledMode = limelightLeft.getNTTable().getEntry("ledMode");
    if (ledMode.getInteger(0) == 1)
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
    isPoseEstimatorReady = true;
  }
}