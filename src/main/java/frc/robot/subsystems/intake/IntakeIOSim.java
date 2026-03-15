package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final SingleJointedArmSim pivotSim;
    private final FlywheelSim rollerSim;

    private final PIDController pivotPID;
    private final PIDController rollerPID;

    private double pivotVolts;
    private double rollerVolts;
    private boolean isClosedLoopPivot;
    private boolean isClosedLoopRoller;


    public IntakeIOSim() {
        pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1),
                IntakeConstants.PIVOT_GEAR_RATIO,
                SingleJointedArmSim.estimateMOI(0.5, 2),
                0.5,
                0,
                Math.toRadians(145.0),
                true,
                0);

        rollerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getNEO(1),
                        IntakeConstants.ROLLER_MOI,
                        IntakeConstants.ROLLER_GEAR_RATIO),
                DCMotor.getNEO(1));

        pivotPID = new PIDController(IntakeConstants.PIVOT_P * 125.0 , IntakeConstants.PIVOT_I, IntakeConstants.PIVOT_D);
        rollerPID = new PIDController(IntakeConstants.ROLLER_P * 10.0, IntakeConstants.ROLLER_I, IntakeConstants.ROLLER_D);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        if (isClosedLoopPivot){
            pivotVolts = pivotPID.calculate(pivotSim.getAngleRads());
        } 
        if (isClosedLoopRoller){
            rollerVolts = rollerPID.calculate(rollerSim.getAngularVelocityRPM());
        }

        pivotSim.setInput(pivotVolts);
        rollerSim.setInput(rollerVolts);

        pivotSim.update(0.02);
        rollerSim.update(0.02);

        inputs.pivotConnected = true;
        inputs.pivotAppliedVolts = pivotVolts;
        inputs.pivotCurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.pivotPositionDeg = Math.toDegrees(pivotSim.getAngleRads());
        inputs.pivotVelocityDegPerSec = Math.toDegrees(pivotSim.getVelocityRadPerSec());

        inputs.rollerConnected = true;
        inputs.rollerAppliedVolts = rollerVolts;
        inputs.rollerCurrentAmps = rollerSim.getCurrentDrawAmps();
        inputs.rollerCurrentRPM = rollerSim.getAngularVelocityRPM();
    }

    public void setPivot(double speed) {
        isClosedLoopPivot = false;
        pivotVolts = speed * 12.0;
    }

    public void setRoller(double speed) {
        isClosedLoopRoller = false;
        rollerVolts = speed * 12.0;
    }

    public void setPivotSetpoint(double deg) {
        isClosedLoopPivot = true;
        pivotPID.setSetpoint(Math.toRadians(deg));
    }

    // at some point we'll make this use a profiled pid controller
    public void setPivotDeg(double deg) {
        isClosedLoopPivot = true;
        pivotPID.setSetpoint(Math.toRadians(deg));
    }

    public void setRollerRPM(double rpm) {
        isClosedLoopRoller = true;
        rollerPID.setSetpoint(rpm);
    }

    public void stopPivot() {
        setPivot(0);
    }

    public void stopRoller() {
        setRoller(0);
    }
}
