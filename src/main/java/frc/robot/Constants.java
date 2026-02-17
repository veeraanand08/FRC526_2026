// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
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
    public static final Rotation3d GYRO_OFFSET = Rotation3d.kZero;
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
    public static final boolean TUNING_MODE_ACTIVE = true;

    public static final int LEFT_SHOOTER_MOTOR = 15;
    public static final int RIGHT_SHOOTER_MOTOR = 16;
    public static final int SHOOTER_CURRENT_LIMIT = 60;
    public static final boolean MOTORS_REVERSED = false;
    public static final double SHOOTER_P = 0.0004;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;
    public static final double SHOOTER_FF = 1.0 / 5676.0;
    public static final double NEGATIVE_RATE_LIMIT = 3000;
    // set speeds
    public static final double DEFAULT_RPM = 3500;
    public static final double REVERSED_RPM = 2500; // reversal if something is stuck

    /* Shoot on the fly */
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

  public static final class FeederConstants
  {
    public static final int LEFT_INDEXER_MOTOR = 18;
    public static final int RIGHT_INDEXER_MOTOR = 19;
    public static final int KICKER_MOTOR = 17;

    public static final int INDEXER_CURRENT_LIMIT = 40;
    public static final int KICKER_CURRENT_LIMIT = 60;
    
    public static final boolean LEFT_INDEXER_MOTOR_REVERSED = false;
    public static final boolean RIGHT_INDEXER_MOTOR_REVERSED = true;
    public static final boolean KICKER_MOTOR_REVERSED = false;

    public static final double KICKER_P = 0.0004;
    public static final double KICKER_I = 0;
    public static final double KICKER_D = 0;
    public static final double KICKER_FF = 1.0 / 5676.0;

    public static final double INDEXER_POWER = 0.8;
    public static final double KICKER_RPM = 5000;
  }
  
  public static final class IntakeConstants
  {
    public static final int ROLLER_MOTOR = 14;
    public static final int ROLLER_CURRENT_LIMIT = 50;
    public static final double ROLLER_POWER = 0.9;
    public static final double ROLLER_POWER_SLOW = 0.6;
    public static final double ROLLER_REVERSED_POWER = -0.7;

    public static final boolean ROLLER_REVERSED = true;
    public static final boolean PIVOT_REVERSED = false;
    
    public static final int PIVOT_MOTOR = 13;
    public static final int PIVOT_CURRENT_LIMIT = 40;
    public static final double PIVOT_GEAR_RATIO = 75;
    public static final double PIVOT_ROT_TO_DEG = 360 / PIVOT_GEAR_RATIO;
    public static final double PIVOT_P = 0.008;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0;
    public static final double PIVOT_FF_S = 0;
    public static final double PIVOT_FF_V = 1.0 / 5676.0;
    public static final double PIVOT_FF_COS = 0; // gravity feedforward
    public static final double PIVOT_FF_COS_RATIO = PIVOT_GEAR_RATIO / 360;

    // MAXMotion
    public static final double PIVOT_CRUISE_VELOCITY = 30; // RPM
    public static final double PIVOT_MAX_ACCEL = 10; // RPM/s
    public static final double ALLOWED_PROFILE_ERROR = 1;
    
    // setpoints, in degrees
    public static final double PIVOT_RAISED_ANGLE = 0;
    public static final double PIVOT_ENGAGED_ANGLE = 130; // lowered
    public static final double PIVOT_AGITATION_UPPER_ANGLE  = 40; 
    public static final double PIVOT_AGITATION_LOWER_ANGLE  = 105;
  }

  public static final class VisionConstants
  {
    public static final String LIMELIGHT_LEFT_NAME = "limelight-left";
    public static final String LIMELIGHT_RIGHT_NAME = "limelight-right";
    public static final double MAX_TAG_AMBIGUITY = Double.MAX_VALUE;
    public static final double MAX_TILT_DEG = 12.0;
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
