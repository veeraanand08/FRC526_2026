package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class contains methods that are used throughout the
 * codebase and are not bound to one subsystem or class.
 */
public final class RobotUtil {
    public static boolean isPoseEstimatorReady;
//    public static ShiftTimer shiftTimer;

    /**
    * Checks if the alliance is red, defaults to false if alliance isn't available.
    *
    * @return true if the red alliance, false if blue. Defaults to false if none is available.
    */
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

//    public static ShiftTimer shiftTimer() {
//        if (shiftTimer == null) {
//            shiftTimer = new ShiftTimer();
//        }
//        return shiftTimer;
//    }

//    private static class ShiftTimer {
//        private enum ShiftSegment {
//            TRANSITION,
//            ALLIANCE,
//            ENDGAME
//        }
//
//        private final Timer timer;
//        private ShiftSegment currentSegment;
//        private int allianceShiftNum;
//        private boolean isHubActive;
//
//        private ShiftTimer() {
//            timer = new Timer();
//            allianceShiftNum = 0;
//        }
//
//        public void start() {
//            currentSegment = ShiftSegment.TRANSITION;
//            isHubActive = true;
//            timer.restart();
//        }
//
//        public void update() {
//            switch (currentSegment) {
//                case TRANSITION:
//                    if (timer.get() >= 10.0) {
//                        char firstInactiveHub = DriverStation.getGameSpecificMessage().charAt(0);
//                        switch (firstInactiveHub) {
//                            case 'B':
//                                isHubActive = isRedAlliance();
//                                break;
//                            case 'R':
//                                isHubActive = !isRedAlliance();
//                                break;
//                            default:
//                                isHubActive = true;
//                        }
//                        currentSegment = ShiftSegment.ALLIANCE;
//                        allianceShiftNum = 1;
//                    }
//                    break;
//                case ALLIANCE:
//                    if (timer.get() >= 25.0) {
//                        allianceShiftNum++;
//                        isHubActive = !isHubActive;
//                        if (allianceShiftNum > 4) {
//                            currentSegment = ShiftSegment.ENDGAME;
//                            isHubActive = true;
//                        }
//                        timer.restart();
//                    }
//                    break;
//                case ENDGAME:
//                    if (timer.get() > 30.0) {
//
//                    }
//            }
//        }
//
//        public void end() {}
//    }
}
