package frc.robot.subsystems.feeder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.FeederConstants;

public class FeederIOSim implements FeederIO {

    private final FlywheelSim leftIndexerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), FeederConstants.ROLLER_MOI, FeederConstants.ROLLER_GEAR_RATIO),
     DCMotor.getNEO(1),
      0);

    private final FlywheelSim rightIndexerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), FeederConstants.ROLLER_MOI, FeederConstants.ROLLER_GEAR_RATIO),
     DCMotor.getNEO(1),
      0);

    private double leftIndexerVolts = 0.0;
    private double rightIndexerVolts = 0.0;

    private final FlywheelSim kickerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), FeederConstants.ROLLER_MOI, FeederConstants.ROLLER_GEAR_RATIO),
     DCMotor.getNEO(1),
      0);

    private final PIDController kickerPID = new PIDController(FeederConstants.KICKER_P * 125.0 , FeederConstants.KICKER_I, FeederConstants.KICKER_D);

    private double kickerVolts = 0.0;
    private boolean isClosedLoopKicker = false;


    public FeederIOSim(){

    }

    public void updateInputs(FeederIOInputs inputs) {
        if (isClosedLoopKicker){
            kickerVolts = kickerPID.calculate(kickerSim.getAngularVelocityRPM());
        }

        kickerSim.setInput(kickerVolts);
        leftIndexerSim.setInput(leftIndexerVolts);
        rightIndexerSim.setInput(rightIndexerVolts);

        kickerSim.update(0.02);
        leftIndexerSim.update(0.02);
        rightIndexerSim.update(0.02);

        inputs.kickerConnected = true;
        inputs.kickerAppliedVolts = kickerVolts;
        inputs.kickerCurrentAmps = kickerSim.getCurrentDrawAmps();
        inputs.kickerCurrentRPM = Math.toDegrees(kickerSim.getAngularVelocityRPM());

        inputs.indexerRightConnected = true;
        inputs.indexerRightAppliedVolts = rightIndexerVolts;
        inputs.indexerRightCurrentAmps = rightIndexerSim.getCurrentDrawAmps();

        inputs.indexerLeftConnected = true;
        inputs.indexerLeftAppliedVolts = leftIndexerVolts;
        inputs.indexerLeftCurrentAmps = leftIndexerSim.getCurrentDrawAmps();

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
        kickerPID.setSetpoint(rpm);
        isClosedLoopKicker = true;
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
