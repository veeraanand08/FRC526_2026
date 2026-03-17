package frc.robot.subsystems.feeder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.FeederConstants;

public class FeederIOSim implements FeederIO {
    private final FlywheelSim leftIndexerSim;
    private final FlywheelSim rightIndexerSim;
    private final FlywheelSim kickerSim;

    private final PIDController kickerPID;

    private double leftIndexerVolts;
    private double rightIndexerVolts;
    private double kickerVolts;
    private boolean isClosedLoopKicker;

    public FeederIOSim() {
        leftIndexerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getNEO(1),
                        FeederConstants.INDEXER_MOI,
                        FeederConstants.KICKER_GEAR_RATIO),
                DCMotor.getNEO(1));

        rightIndexerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getNEO(1),
                        FeederConstants.INDEXER_MOI,
                        FeederConstants.KICKER_GEAR_RATIO),
                DCMotor.getNEO(1));

        kickerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getNEO(1),
                        FeederConstants.KICKER_MOI,
                        FeederConstants.KICKER_GEAR_RATIO),
                DCMotor.getNEO(1));

        kickerPID = new PIDController(FeederConstants.KICKER_P * 125.0 , FeederConstants.KICKER_I, FeederConstants.KICKER_D);
    }

    public void updateInputs(FeederIOInputs inputs) {
        if (isClosedLoopKicker) {
            kickerVolts = kickerPID.calculate(kickerSim.getAngularVelocityRPM());
        }

        leftIndexerSim.setInput(leftIndexerVolts);
        rightIndexerSim.setInput(rightIndexerVolts);
        kickerSim.setInput(kickerVolts);

        leftIndexerSim.update(0.02);
        rightIndexerSim.update(0.02);
        kickerSim.update(0.02);

        inputs.indexerRightConnected = true;
        inputs.indexerRightAppliedVolts = rightIndexerVolts;
        inputs.indexerRightCurrentAmps = rightIndexerSim.getCurrentDrawAmps();

        inputs.indexerLeftConnected = true;
        inputs.indexerLeftAppliedVolts = leftIndexerVolts;
        inputs.indexerLeftCurrentAmps = leftIndexerSim.getCurrentDrawAmps();

        inputs.kickerConnected = true;
        inputs.kickerAppliedVolts = kickerVolts;
        inputs.kickerCurrentAmps = kickerSim.getCurrentDrawAmps();
        inputs.kickerCurrentRPM = kickerSim.getAngularVelocityRPM();
    }

    public void setIndexerLeft(double speed) {
        leftIndexerVolts = speed * 12.0;
    }

    public void setIndexerRight(double speed) {
        rightIndexerVolts = speed * 12.0;
    }

    public void setKicker(double speed) {
        isClosedLoopKicker = false;
        kickerVolts = speed * 12.0;
    }

    public void setKickerRPM(double rpm) {
        isClosedLoopKicker = true;
        kickerPID.setSetpoint(rpm);
    }

    public void stopIndexerLeft() {
        leftIndexerVolts = 0.0;
    }

    public void stopIndexerRight() {
        rightIndexerVolts = 0.0;
    }

    public void stopKicker() {
        setKicker(0);
    }


}
