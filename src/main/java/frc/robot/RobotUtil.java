package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class contains methods that are used throughout the
 * codebase and are not bound to one subsystem or class.
 */
public final class RobotUtil {
    public static boolean isPoseEstimatorReady;
    public final static ShiftTimer shiftTimer = new ShiftTimer();

    /**
    * Checks if the alliance is red, defaults to false if alliance isn't available.
    *
    * @return true if the red alliance, false if blue. Defaults to false if none is available.
    */
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    public static class ShiftTimer {
        private enum ShiftSegment {
            TRANSITION,
            ALLIANCE,
            ENDGAME
        }

        private final Timer timer;
        private ShiftSegment currentSegment;
        private int allianceShiftNum;
        private boolean isHubActive;
        private double shiftTimeRemaining;

        private ShiftTimer() {
            timer = new Timer();
            SmartDashboard.setDefaultBoolean("Match/Hub Active", false);
            SmartDashboard.setDefaultString("Match/Current Shift", "N/A");
            SmartDashboard.setDefaultNumber("Match/Shift Timer", 0.0);
        }

        public void start() {
            timer.restart();
            currentSegment = ShiftSegment.TRANSITION;
            allianceShiftNum = 0;
            isHubActive = true;
            SmartDashboard.putBoolean("Match/Hub Active", isHubActive);
            SmartDashboard.putString("Match/Current Shift", currentSegment.toString());
        }

        public void update() {
            switch (currentSegment) {
                case TRANSITION:
                    if (timer.get() >= 10.0) {
                        timer.restart();
                        String gameData = DriverStation.getGameSpecificMessage();
                        char firstInactiveHub;
                        if (!gameData.isEmpty())
                            firstInactiveHub = gameData.charAt(0);
                        else firstInactiveHub = ' ';
                        switch (firstInactiveHub) {
                            case 'B':
                                isHubActive = isRedAlliance();
                                break;
                            case 'R':
                                isHubActive = !isRedAlliance();
                                break;
                            default:
                                isHubActive = true;
                        }
                        currentSegment = ShiftSegment.ALLIANCE;
                        allianceShiftNum = 1;
                        SmartDashboard.putString("Match/Current Shift", currentSegment.toString() + " " + allianceShiftNum);
                        break;
                    }
                    shiftTimeRemaining = 11.0 - timer.get();
                    break;
                case ALLIANCE:
                    if (timer.get() >= 25.0) {
                        timer.restart();
                        allianceShiftNum++;
                        isHubActive = !isHubActive;
                        if (allianceShiftNum > 4) {
                            currentSegment = ShiftSegment.ENDGAME;
                            isHubActive = true;
                            SmartDashboard.putString("Match/Current Shift", currentSegment.toString());
                        }
                        else SmartDashboard.putString("Match/Current Shift", currentSegment.toString() + " " + allianceShiftNum);
                        break;
                    }
                    shiftTimeRemaining = 26.0 - timer.get();
                    break;
                case ENDGAME:
                    shiftTimeRemaining = 31.0 - timer.get();
                    if (timer.get() >= 30.0) {
                        end();
                    }
                    break;
            }
            SmartDashboard.putBoolean("Match/Hub Active", isHubActive);
            SmartDashboard.putNumber("Match/Shift Timer", shiftTimeRemaining);
        }

        public void end() {
            timer.stop();
            isHubActive = false;
            SmartDashboard.putBoolean("Match/Hub Active", false);
            SmartDashboard.putString("Match/Current Shift", "N/A");
            SmartDashboard.putNumber("Match/Shift Timer", 0.0);
        }

        public double getTimeRemaining() {
            return shiftTimeRemaining;
        }
    }
}
