package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        // pivot
        public boolean pivotConnected = false;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotPositionDeg = 0.0;
        public double pivotVelocityDegPerSec = 0.0;

        // roller
        public boolean rollerConnected = false;
        public double rollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;
        public double rollerCurrentRPM = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Set the motor's idle mode */
    public default void setPivotBrake(boolean brake) {}

    /** Sets the motor's voltage given a percentage input from -1.0 to 1.0 */
    public default void setPivot(double speed) {}

    public default void setRoller(double speed) {}

    /** Sets the intake pivot's angle given a degree input */
    public default void setPivotDeg(double deg) {}

    /** Sets the motor's speed given an RPM input */
    public default void setRollerRPM(double rpm) {}

    /** Stop the motor */
    public default void stopPivot() {}

    public default void stopRoller() {}
}