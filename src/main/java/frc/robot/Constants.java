// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
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
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class TrenchAlignmentConstants
  {
    public static Translation2d RED_LEFT_TRENCH = new Translation2d(11.928, 0.586);
    public static Translation2d RED_RIGHT_TRENCH = new Translation2d(11.928, 7.423);
    public static Translation2d BLUE_LEFT_TRENCH = new Translation2d(4.617, 0.586);
    public static Translation2d BLUE_RIGHT_TRENCH = new Translation2d(4.617, 7.423);

    public static final double TRENCH_ALIGNMENT_THRESHOLD = 1.5;

    public static final double Y_P = 0.0025;
    public static final double Y_I = 0.0;
    public static final double Y_D = 0.0;

    public static final double ANGLE_P = 0.0025;
    public static final double ANGLE_I = 0.0;
    public static final double ANGLE_D = 0.0;


    public static final double TRENCH_ROTATION_SETPOINT = 0.0;

    public static final double STRENGTH_EXP = 3.0;

  }



  public static final class DrivebaseConstants
  {
    public static final Rotation3d GYRO_OFFSET = Rotation3d.kZero;
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(3)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    // Maximum speed of the robot in meters per second, used to limit acceleration.
    public static final double MAX_SPEED  = 4;
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
    public static final boolean TUNING_MODE_ACTIVE = false;

    public static final int LEFT_SHOOTER_MOTOR = 15;
    public static final int RIGHT_SHOOTER_MOTOR = 16;
    public static final int SHOOTER_CURRENT_LIMIT = 60;
    public static final boolean MOTORS_REVERSED = false;
    public static final double SHOOTER_P = 0.00004;
    public static final double SHOOTER_I = 0;//0.0000003;
    public static final double SHOOTER_D = 0;
    public static final double SHOOTER_FF = 1.0 / 5676.0;
    public static final double NEGATIVE_RATE_LIMIT = 2000;
    // set speeds
    public static final double DEFAULT_RPM = 3500;
    public static final double REVERSED_RPM = 2500; // reversal if something is stuck

    /* Shoot on the fly */
    public static final double MAX_DISTANCE = Double.MAX_VALUE; //temp
    public static final int MAX_ITERATIONS = 3;

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME = new InterpolatingDoubleTreeMap();
    static {
      // Distance, Time
      DISTANCE_TO_TIME.put(6.730068956, 1.786);
      DISTANCE_TO_TIME.put(6.472743156, 1.733);
      DISTANCE_TO_TIME.put(6.044155496, 1.693);
      DISTANCE_TO_TIME.put(5.506356383, 1.483);
      DISTANCE_TO_TIME.put(5.068445123, 1.4);
      DISTANCE_TO_TIME.put(4.515344442, 1.35);
      DISTANCE_TO_TIME.put(4.182420309, 1.25);
      DISTANCE_TO_TIME.put(3.673340232, 1.25);
      DISTANCE_TO_TIME.put(2.972515343, 1.283);
      DISTANCE_TO_TIME.put(2.425870923, 1.133);
      DISTANCE_TO_TIME.put(1.915421252, 0.957);
    }
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_RPM = new InterpolatingDoubleTreeMap();
    static {
      // Distance, RPM for Shot
      DISTANCE_TO_RPM.put(6.730068956, 4500d);
      DISTANCE_TO_RPM.put(6.472743156, 4300d);
      DISTANCE_TO_RPM.put(6.044155496, 4100d);
      DISTANCE_TO_RPM.put(5.506356383, 3850d);
      DISTANCE_TO_RPM.put(5.068445123, 3675d);
      DISTANCE_TO_RPM.put(4.515344442, 3500d);
      DISTANCE_TO_RPM.put(4.182420309, 3325d);
      DISTANCE_TO_RPM.put(3.673340232, 3150d);
      DISTANCE_TO_RPM.put(2.972515343, 3000d);
      DISTANCE_TO_RPM.put(2.425870923, 2675d);
      DISTANCE_TO_RPM.put(1.915421252, 2350d);
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
    public static final double KICKER_RPM_REVERSED = -3000;

    public static final double INDEXER_PERIOD = 1.0;
  }
  
  public static final class IntakeConstants
  {
    public static final int ROLLER_MOTOR = 14;
    public static final int ROLLER_CURRENT_LIMIT = 60;
    public static final double ROLLER_RPM = 5000;
    public static final double ROLLER_RPM_REVERSED = -4500;
    public static final double ROLLER_POWER_SLOW = 0.75;
    public static final double ROLLER_P = 0.0003;
    public static final double ROLLER_I = 0;
    public static final double ROLLER_D = 0;
    public static final double ROLLER_FF = 1.0 / 5676.0;

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
    public static final double PIVOT_ENGAGED_ANGLE = 140; // lowered
    public static final double PIVOT_AGITATION_UPPER_ANGLE  = 40; 
    public static final double PIVOT_AGITATION_LOWER_ANGLE  = 105;

    public static final double AGITATION_PERIOD = 2000; //Period is number * 2 in seconds
  }

  public static final class VisionConstants
  {
    // Camera names, must match names configured on coprocessor
    public static final String CAMERA_0_NAME = "limelight-left";
    public static final String CAMERA_1_NAME = "limelight-right";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
            new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 =
            new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.3;
    public static final double MAX_Z_ERROR = 0.75;
    public static final double MAX_TILT_DEG = 15;

    // Standard deviation baselines, for 1-meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.1; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[] {
                    1.0, // Camera 0
                    1.0 // Camera 1
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class FieldConstants
  {
    // AprilTag layout
    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final Translation2d RED_HUB = new Translation2d(11.938, 4.0);
    public static final Translation2d BLUE_HUB = new Translation2d(4.597, 4.0);
    public static final Translation2d RED_LEFT_BUMP = new Translation2d(11.928, 2.408);
    public static final Translation2d RED_RIGHT_BUMP = new Translation2d(11.928, 5.598);
    public static final Translation2d BLUE_LEFT_BUMP = new Translation2d(4.617, 5.598);
    public static final Translation2d BLUE_RIGHT_BUMP = new Translation2d(4.617, 2.408);

    public static final double RED_ALLIANCE_BOUNDARY = RED_LEFT_BUMP.getX();
    public static final double BLUE_ALLIANCE_BOUNDARY = BLUE_LEFT_BUMP.getX();
  }

  public static final class ControllerConstants
  {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    // Joystick Deadband
    public static final double DEADBAND         = 0.025;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}