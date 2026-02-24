package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class contains methods that are used throughout the
 * codebase and are not bound to one subsystem or class.
 */
public final class RobotUtil {
    public static boolean isPoseEstimatorReady;

    /**
    * Checks if the alliance is red, defaults to false if alliance isn't available.
    *
    * @return true if the red alliance, false if blue. Defaults to false if none is available.
    */
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}
