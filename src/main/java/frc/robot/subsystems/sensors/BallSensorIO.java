package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface BallSensorIO {
    @AutoLog
    public static class FeederIOInputs {
        public double distanceMeters = 0.0;
        public boolean valid = false;
    }

    public default void updateInputs(FeederIOInputs inputs) {}
}