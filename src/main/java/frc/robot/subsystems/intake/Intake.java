package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotUtil;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public enum PivotState {
    RAISING,
    AGITATING,
    LOWERING
  }

  private PivotState pivotState = PivotState.RAISING;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean rollerEnabled;

  private final Timer agitationTimer;

  public Intake(IntakeIO io) {
    this.io = io;

    agitationTimer = new Timer();

    Logger.recordOutput("Intake/Pivot State", pivotState.toString());
    Logger.recordOutput("Intake/Intake Running", rollerEnabled);
  }

  @Override
  /* Periodically raises/lowers the pivot depending on its current state. Will not run if in lowered/lowering state. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (pivotState == PivotState.AGITATING) {
      // safety check
      if (isPivotStalled()) setPivotState(PivotState.LOWERING);

      double time = agitationTimer.get();

      double pos = Math.sin(time * 2 * Math.PI / IntakeConstants.AGITATION_PERIOD) * 0.5 + 0.5;

      double upperAngle = IntakeConstants.PIVOT_AGITATION_UPPER_ANGLE
              -  (IntakeConstants.PIVOT_AGITATION_UPPER_ANGLE - IntakeConstants.PIVOT_AGITATION_UPPER_ANGLE_MIN) * (time / IntakeConstants.PIVOT_UPPER_AGITATION_DECAY_TIME);

      // Clamp so it never goes past the lower angle
      upperAngle = Math.max(upperAngle, IntakeConstants.PIVOT_AGITATION_UPPER_ANGLE_MIN);

      double targetAngle = upperAngle + (IntakeConstants.PIVOT_AGITATION_LOWER_ANGLE - upperAngle) * pos;

      io.setPivotSetpoint(targetAngle);
      Logger.recordOutput("Intake/Pivot Setpoint", targetAngle);
    }
  }

  public void setPivotState(PivotState newState) {
    switch (newState){
      case RAISING:
        agitationTimer.stop();
        setPivotAngle(IntakeConstants.PIVOT_RAISED_ANGLE);
        break;
      case LOWERING:
        agitationTimer.stop();
        setPivotAngle(IntakeConstants.PIVOT_ENGAGED_ANGLE);
        break;
      case AGITATING:
        agitationTimer.restart();
        break;
      default:
        break;
    }
    pivotState = newState;
    Logger.recordOutput("Intake/Pivot State", pivotState.toString());
  }

  public void toggleRoller() {
    setRoller(!rollerEnabled);
  }

  public void setRoller(boolean enabled) {
    if (enabled) io.setRollerRPM(IntakeConstants.ROLLER_RPM);
    else io.stopRoller();
    rollerEnabled = enabled;
    Logger.recordOutput("Intake/Intake Running", rollerEnabled);
  }

  public void slowRoller() {
    io.setRollerRPM(IntakeConstants.ROLLER_RPM_SLOW);
    rollerEnabled = false;
    Logger.recordOutput("Intake/Intake Running", false);
  }

  public void setRollerReversed(boolean enabled) {
    if (enabled) io.setRollerRPM(IntakeConstants.ROLLER_RPM_REVERSED);
    else io.stopRoller();
    rollerEnabled = false;
    Logger.recordOutput("Intake/Intake Running", false);
  }

  /**
   * Set the pivot motor's setpoint to a given angle.
   * @param angle Angle (in degrees) to rotate.
   */
  public void setPivotAngle(double angle) {
    io.setPivotDeg(angle);
    Logger.recordOutput("Intake/Pivot Setpoint", angle);
  }

  /* Set the pivot and roller motor speeds to 0. */
  public void stop() {
    setRoller(false);
    io.stopPivot();
  }

  public void setPivotBrake(boolean brake) {
    io.setPivotBrake(brake);
  }

  public boolean isPivotStalled() {
    return inputs.pivotCurrentAmps > IntakeConstants.PIVOT_CURRENT_LIMIT &&
            Math.abs(inputs.pivotVelocityDegPerSec) < IntakeConstants.PIVOT_STALL_VELOCITY;
  }

  public boolean isRollerStalled() {
    return inputs.rollerCurrentAmps > IntakeConstants.ROLLER_CURRENT_LIMIT &&
            Math.abs(inputs.rollerCurrentRPM) < IntakeConstants.ROLLER_STALL_VELOCITY;
  }

  /**
   * Enable the intake roller. If the intake is raised, it will lower and then start.
   *
   * @return a command to toggle the intake
   */
  public Command intakeCommand() {
    // Inline construction of command goes here.
    return startRun(
            () -> {
              if (pivotState == PivotState.LOWERING) {
                setRoller(true);
              }
              else {
                setRoller(true);
                setPivotState(PivotState.LOWERING);
              }
            },
            () -> {
              if (isRollerStalled()) {
                RobotUtil.setOperatorRumble(0.7, 0.7);
              }
            })
            .finallyDo(() -> {
              RobotUtil.setOperatorRumble(0.0, 0.0);
              setRoller(false);
            });
  }

  /**
   * Toggle the intake roller. If the intake is raised, it will lower and then start.
   *
   * @return a command to toggle the intake
   */
  public Command toggleIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          if (pivotState == PivotState.LOWERING) {
            toggleRoller();
          }
          else {
            setRoller(true);
            setPivotState(PivotState.LOWERING);
          }
        });
  }
  
  /**
   * Reverse the intake roller while held
   *
   * @return a command to reverse the intake
   */
  public Command reverseIntakeCommand() {
    // Inline construction of command goes here.
    return startEnd(
      () -> setRollerReversed(true), 
      () -> setRollerReversed(false)
    );
  }

  /**
   * Toggle intake agitation
   *
   * @return a command to agitate the intake
   */
  public Command agitateCommand() {
    return runOnce(() -> {
      if (pivotState == PivotState.AGITATING) {
        setRoller(false);
        setPivotState(PivotState.LOWERING);
      }
      else {
        setPivotState(PivotState.AGITATING);
        slowRoller();
      }
    });
  }

  /**
   * Stop the roller motor and bring the intake back up
   *
   * @return a command to reset the intake back to starting position
   */
  public Command resetIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setRoller(false);
          setPivotState(PivotState.RAISING);
        });
  }

  @Override
  public void simulationPeriodic() {}
}