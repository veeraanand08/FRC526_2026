// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotState;

@SuppressWarnings("unused")
public class IntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  public IntakeCommand(IntakeSubsystem subsys) {
    intakeSubsystem = subsys;
    addRequirements(intakeSubsystem);
  }

  @Override
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
  public void end(boolean interrupted) {
    if (interrupted) intakeSubsystem.stop();
  }
 
  @Override
  public boolean isFinished() {
    return (intakeSubsystem.pivotState == PivotState.LOWERED || intakeSubsystem.pivotState == PivotState.RAISED_LOWERING);
  }
}
