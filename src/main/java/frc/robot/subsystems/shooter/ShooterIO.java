package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // leader
        public boolean leaderConnected = false;
        public double leaderAppliedVolts = 0.0;
        public double leaderCurrentAmps = 0.0;
        public double leaderCurrentRPM = 0.0;

        // follower
        public boolean followerConnected = false;
        public double followerAppliedVolts = 0.0;
        public double followerCurrentAmps = 0.0;
        public double followerCurrentRPM = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Sets the motor's voltage given a percentage input from -1.0 to 1.0 */
    public default void set(double speed) {}

    /** Sets the motor's speed given an RPM input */
    public default void setRPM(double rpm) {}

    /** Stop the motor */
    public default void stop() {}
}