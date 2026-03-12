package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface BallSensorIO {
    @AutoLog
    public static class BallSensorIOInputs {
        public double distanceMiliMeters = 0.0;
        public boolean valid = false;
    }

    public default void updateInputs(BallSensorIOInputs inputs) {}
}