// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotUtil;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoAlign.Target;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignOnce extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerveSubsystem;
  private final Target target;
  private Rotation2d angleToTarget = Rotation2d.kZero;

  /**
   * Creates a new AutoAlign.
   *
   * @param swerveSubsystem The swerve subsystem used by this command.
   * @param target The target that this command aligns to.
   */
  public AutoAlignOnce(SwerveSubsystem swerveSubsystem, Target target)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d robotTranslation = swerveSubsystem.getPose().getTranslation();
    Translation2d targetTranslation = AutoAlign.getTargetTranslation(target, robotTranslation);

    Translation2d difference = targetTranslation.minus(robotTranslation);
    angleToTarget = new Rotation2d(difference.getX(), difference.getY());

    swerveSubsystem.driveFieldOriented(swerveSubsystem.rotateToAngle( 
        0,
        0,
        angleToTarget
    ));
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // cancel command if pose estimator may not be accurate OR if bot is pointing towards hub
    return Math.abs(swerveSubsystem.getHeading().getDegrees() - angleToTarget.getDegrees()) < Constants.AutoAlignConstants.TOLERANCE_DEG;
  }
}
