package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

/**
 * This class contains methods that are used throughout the
 * codebase and are not bound to one subsystem or class.
 */
public final class RobotUtil {
    public static boolean isPoseEstimatorReady;
    public final static ShiftTimer shiftTimer = new ShiftTimer();
    private static CommandXboxController driverController;
    private static CommandXboxController operatorController;

    /**
    * Checks if the alliance is red, defaults to false if alliance isn't available.
    *
    * @return true if the red alliance, false if blue. Defaults to false if none is available.
    */
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    /**
     * Instantiate the Xbox controllers.
     */
    public static void setDriverController(CommandXboxController driverControllerInstance) {
        driverController = driverControllerInstance;
    }

    public static void setOperatorController(CommandXboxController operatorControllerInstance) {
        operatorController = operatorControllerInstance;
    }

    /**
     * Set the Rumble value for each Xbox controller.
     */
    public static void setDriverRumble(double leftRumble, double rightRumble) {
        driverController.setRumble(GenericHID.RumbleType.kLeftRumble, leftRumble);
        driverController.setRumble(GenericHID.RumbleType.kRightRumble, rightRumble);
    }

    public static void setOperatorRumble(double leftRumble, double rightRumble) {
        operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, leftRumble);
        operatorController.setRumble(GenericHID.RumbleType.kRightRumble, rightRumble);
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
            Logger.recordOutput("Match/Hub Active", false);
            Logger.recordOutput("Match/Current Shift", "N/A");
            Logger.recordOutput("Match/Shift Timer", 0.0);
        }

        public void start() {
            timer.restart();
            currentSegment = ShiftSegment.TRANSITION;
            allianceShiftNum = 0;
            isHubActive = true;
            Logger.recordOutput("Match/Hub Active", isHubActive);
            Logger.recordOutput("Match/Current Shift", currentSegment.toString());
        }

        public void update() {
            switch (currentSegment) {
                case TRANSITION:
                    if (timer.hasElapsed(10.0)) {
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
                        Logger.recordOutput("Match/Current Shift", currentSegment.toString() + " " + allianceShiftNum);
                        break;
                    }
                    shiftTimeRemaining = 10.0 - timer.get();
                    break;
                case ALLIANCE:
                    if (timer.hasElapsed(25.0)) {
                        timer.restart();
                        allianceShiftNum++;
                        isHubActive = !isHubActive;
                        if (allianceShiftNum > 4) {
                            currentSegment = ShiftSegment.ENDGAME;
                            isHubActive = true;
                            Logger.recordOutput("Match/Current Shift", currentSegment.toString());
                        }
                        else Logger.recordOutput("Match/Current Shift", currentSegment.toString() + " " + allianceShiftNum);
                        break;
                    }
                    shiftTimeRemaining = 25.0 - timer.get();
                    break;
                case ENDGAME:
                    shiftTimeRemaining = 30.0 - timer.get();
                    if (timer.hasElapsed(30.0)) {
                        end();
                    }
                    break;
            }
            Logger.recordOutput("Match/Hub Active", isHubActive);
            Logger.recordOutput("Match/Shift Timer", shiftTimeRemaining + 1);
        }

        public void end() {
            timer.stop();
            isHubActive = false;
            Logger.recordOutput("Match/Hub Active", false);
            Logger.recordOutput("Match/Current Shift", "N/A");
            Logger.recordOutput("Match/Shift Timer", 0.0);
        }

        public double getTimeRemaining() {
            return shiftTimeRemaining;
        }
    }
}
