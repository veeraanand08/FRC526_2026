package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {

    private final FlywheelSim shooterSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), ShooterConstants.SHOOTER_MOI, ShooterConstants.SHOOTER_GEAR_RATIO), 
    DCMotor.getNEO(1)
    );

    private PIDController shooterPID = new PIDController(ShooterConstants.SHOOTER_P * 125.0, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D);
    
    private double shooterVolts = 0.0;
    private boolean isClosedLoopShooter = false;

    public ShooterIOSim(){}

    public void updateInputs(ShooterIOInputs inputs) {
        if (isClosedLoopShooter){
            shooterVolts = shooterPID.calculate(shooterSim.getAngularVelocityRPM());
        }

        shooterSim.setInput(shooterVolts);
        shooterSim.update(0.02);

        inputs.leaderConnected = true;
        inputs.leaderAppliedVolts = shooterVolts;
        inputs.leaderCurrentAmps = shooterSim.getCurrentDrawAmps();
        inputs.leaderCurrentRPM = shooterSim.getAngularVelocityRPM();


    }

    public void set(double speed) {
        isClosedLoopShooter = false;
        shooterVolts = speed * 12.0;

    }

    public void setRPM(double rpm) {
        isClosedLoopShooter = true;
        shooterPID.setSetpoint(rpm);
    }

    public void stop() {
        set(0.0);
    }
}
