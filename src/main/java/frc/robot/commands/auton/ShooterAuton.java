// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAuton extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooterSubsystem;
  private final FeederSubsystem feederSubsystem;
//  private final IntakeSubsystem intakeSubsystem;
  private final boolean isReversed;
  private final Timer timer;
  private boolean startShoot;

  /**
   * Creates a new ShooterCommand.
   *
   * @param shooterSubsystem The shooter subsystem used by this command.
   * @param feederSubsystem The feeder subsystem used by this command.
   * // @param intakeSubsystem The intake subsystem used by this command.
   * @param reversed Whether or not to run this command in reversed mode to get a ball unstuck.
   */
  public ShooterAuton(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, /*IntakeSubsystem intakeSubsystem,*/
                        boolean reversed)
  {
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
//    this.intakeSubsystem = intakeSubsystem;
    this.isReversed = reversed;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, feederSubsystem/*, intakeSubsystem*/);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startShoot = false;
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.shoot();
    if (!startShoot && shooterSubsystem.hasSpunUp()) {
      startShoot = true;
      feederSubsystem.enableKicker();
      feederSubsystem.enableIndexer(isReversed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stop();
    shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
