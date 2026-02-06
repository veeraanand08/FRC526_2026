// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final class DrivebaseConstants
  {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(3)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    // Maximum speed of the robot in meters per second, used to limit acceleration.
    public static final double MAX_SPEED  = 3.5;
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  // public static final class AutonConstants
  // {
  //   public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //   public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class ShooterConstants
  {
    public static final int LEFT_SHOOTER_MOTOR = 13;
    public static final int RIGHT_SHOOTER_MOTOR = 14;
    public static final boolean LEFT_SHOOTER_MOTOR_REVERSED = false;
    public static final boolean RIGHT_SHOOTER_MOTOR_REVERSED = false;
    public static final double SHOOTER_P = 0;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;
    public static final double SHOOTER_GEAR_RATIO = 1;

    // Shoot on the fly
    public static final double MAX_DISTANCE = Double.MAX_VALUE; //temp
    public static final int MAX_ITERATIONS = 0;

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME = new InterpolatingDoubleTreeMap();
    static {
      // Distance, Time
      DISTANCE_TO_TIME.put(3.0, 1.0); //Example
    }
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_RPM = new InterpolatingDoubleTreeMap();
    static {
      // Distance, RPM for Shot
      DISTANCE_TO_RPM.put(1.0, 1.0); //Example
    }
  }

  public static final class FieldConstants
  {
    public static final Translation2d RED_HUB = new Translation2d(11.938, 4.0);
    public static final Translation2d BLUE_HUB = new Translation2d(4.597, 4.0);
    public static final Translation2d RED_LEFT_BUMP = new Translation2d(11.928, 2.408);
    public static final Translation2d RED_RIGHT_BUMP = new Translation2d(11.928, 5.598);
    public static final Translation2d BLUE_LEFT_BUMP = Translation2d.kZero; // temp
    public static final Translation2d BLUE_RIGHT_BUMP = Translation2d.kZero; // temp
  }

  public static final class ControllerConstants
  {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    // Joystick Deadband
    public static final double DEADBAND         = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
