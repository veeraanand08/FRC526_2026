package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotState;

public class IntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;

  public IntakeCommand(IntakeSubsystem subsystem) {
    intakeSubsystem = subsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  /* If in engaged position disable roller and bring it up to raised, otherwise set it to lowering */
  public void initialize() {}

  @Override
  public void execute() {
    switch (intakeSubsystem.pivotState) {
      case LOWERED:
        intakeSubsystem.pivotState = PivotState.SHAKING_UP;
        intakeSubsystem.setRoller(false);
        break;
      default:
        intakeSubsystem.pivotState = PivotState.LOWERING;
    }

  }

  @Override
  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {}
 
  @Override
  /* The process is finished if the pivot is in the engaged position or is lowering in the raised state */
  public boolean isFinished() {
    return (intakeSubsystem.pivotState == PivotState.LOWERED || intakeSubsystem.pivotState == PivotState.SHAKING_DOWN);
  }
}
