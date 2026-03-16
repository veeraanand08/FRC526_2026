package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        // left side
        public boolean indexerLeftConnected = false;
        public double indexerLeftAppliedVolts = 0.0;
        public double indexerLeftCurrentAmps = 0.0;

        // right side
        public boolean indexerRightConnected = false;
        public double indexerRightAppliedVolts = 0.0;
        public double indexerRightCurrentAmps = 0.0;

        // kicker
        public boolean kickerConnected = false;
        public double kickerAppliedVolts = 0.0;
        public double kickerCurrentAmps = 0.0;
        public double kickerCurrentRPM = 0.0;
    }

    public default void updateInputs(FeederIOInputs inputs) {}

    /** Sets the motor's voltage given a percentage input from -1.0 to 1.0 */
    public default void setIndexerLeft(double speed) {}

    public default void setIndexerRight(double speed) {}

    public default void setKicker(double speed) {}

    /** Sets the motor's speed given an RPM input */
    public default void setKickerRPM(double rpm) {}

    /** Stop the motor */
    public default void stopIndexerLeft() {}

    public default void stopIndexerRight() {}

    public default void stopKicker() {}
}