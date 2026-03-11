// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterFallback extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooterSubsystem;
  private final Feeder feederSubsystem;
  private boolean startShoot;

  /**
   * Creates a new ShooterFallback Command.
   * This command uses a setpoint RPM rather than dynamically adjusting based on distance.
   *
   * @param shooterSubsystem The shooter subsystem used by this command.
   * @param feederSubsystem The feeder subsystem used by this command.
   */
  public ShooterFallback(Shooter shooterSubsystem, Feeder feederSubsystem)
  {
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, feederSubsystem/*, intakeSubsystem*/);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startShoot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setAngularVelocity(ShooterConstants.DEFAULT_RPM.get());
    if (!startShoot && shooterSubsystem.hasSpunUp()) {
      startShoot = true;
      feederSubsystem.enableKicker();
      feederSubsystem.enableIndexer(false);
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
