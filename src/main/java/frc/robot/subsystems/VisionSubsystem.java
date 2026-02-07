// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  
  public double getLimelightAngle() {
    NetworkTable leftTable = NetworkTableInstance.getDefault().getTable("limelight-left");
    double tVisibleLeft = leftTable.getEntry("tv").getDouble(0);
    double tDegreeOffsetLeft = leftTable.getEntry("tx").getDouble(0);
    NetworkTable leftRight = NetworkTableInstance.getDefault().getTable("limelight-right");
    double tVisibleRight = leftRight.getEntry("tv").getDouble(0);
    double tDegreeOffsetRight = leftRight.getEntry("tx").getDouble(0);
    
    if (tVisibleLeft == 1 && tVisibleRight == 1){
      return (tDegreeOffsetLeft + tDegreeOffsetRight)/2.0;
    } else if (tVisibleLeft == 1) {
      return tDegreeOffsetLeft;
    } else if (tVisibleRight == 1){
      return tDegreeOffsetRight;
    } else {
      return 0.0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
