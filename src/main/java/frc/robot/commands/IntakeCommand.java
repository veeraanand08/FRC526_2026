package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotState;


public class IntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  public IntakeCommand(IntakeSubsystem subsys) {
    intakeSubsystem = subsys;
    addRequirements(intakeSubsystem);
  }

  @Override
  /* If in engaged position disable roller and bring it up to raised, otherwise set it to lowering */
  public void initialize() {
    if (intakeSubsystem.pivotState==PivotState.LOWERED) {
      intakeSubsystem.pivotState=PivotState.RAISED_RAISING;
      intakeSubsystem.toggleRoller();
    } else intakeSubsystem.pivotState=PivotState.LOWERING;
  }

  @Override
  public void execute() {
    double currentDeg = intakeSubsystem.getPivotDeg();
    if (intakeSubsystem.pivotState==PivotState.LOWERING) {
      intakeSubsystem.setPivotPos(ModuleConstants.INTAKE_ENGAGED_ANGLE);
      if (currentDeg>=ModuleConstants.INTAKE_ENGAGED_ANGLE-5) {
        intakeSubsystem.pivotState=PivotState.LOWERED;
        intakeSubsystem.toggleRoller();
        intakeSubsystem.stopPivot();
      }
    } else if (currentDeg<=ModuleConstants.INTAKE_UPPER_RAISED+5) {
      intakeSubsystem.pivotState=PivotState.RAISED_LOWERING;
    }
  }

  @Override
  /* Only stop everything if there was an issue / interrupted */
  public void end(boolean interrupted) {
    if (interrupted) intakeSubsystem.stop();
  }
 
  @Override
  /* The process is finished if the pivot is in the engaged position or is lowering in the raised state */
  public boolean isFinished() {
    return (intakeSubsystem.pivotState == PivotState.LOWERED || intakeSubsystem.pivotState == PivotState.RAISED_LOWERING);
  }
}
