// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TrenchAlignmentConstants;
import frc.robot.RobotUtil;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;;

public class TrenchAlign extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerveSubsystem;
  private final CommandXboxController driverController;
  private Translation2d trenchTarget;
  private final PIDController yController;
  private final PIDController angleController;


  /**
   * Creates a new AutoAlign.
   *
   * @param swerveSubsystem The swerve subsystem used by this command.
   * @param driverController The CommandXboxController object of the driver's controller.
   */
  public TrenchAlign(SwerveSubsystem swerveSubsystem,
    CommandXboxController driverController)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.driverController = driverController;
    this.trenchTarget = Translation2d.kZero;
    yController = new PIDController(TrenchAlignmentConstants.Y_P, TrenchAlignmentConstants.Y_I, TrenchAlignmentConstants.Y_D);
    angleController = new PIDController(TrenchAlignmentConstants.ANGLE_P, TrenchAlignmentConstants.ANGLE_I, TrenchAlignmentConstants.ANGLE_D);
    
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trenchTarget = swerveSubsystem.trenchTarget();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = (RobotUtil.isRedAlliance()) ? driverController.getLeftY() : -driverController.getLeftY();
    double leftX = (RobotUtil.isRedAlliance()) ? driverController.getLeftX() : -driverController.getLeftX();
    double angularVelocityController = (RobotUtil.isRedAlliance()) ? driverController.getRightX() : -driverController.getRightX();

    if (!trenchTarget.equals(Translation2d.kZero)){
      double ySpeed = yController.calculate(swerveSubsystem.getPose().getY(), trenchTarget.getY());

      
      double angleSpeed = angleController.calculate(swerveSubsystem.getHeading().getRadians(), TrenchAlignmentConstants.TRENCH_ROTATION_SETPOINT);
      
      double strength = Math.pow(1 - (swerveSubsystem.getPose().getTranslation().getDistance(trenchTarget)  / TrenchAlignmentConstants.TRENCH_ALIGNMENT_THRESHOLD), TrenchAlignmentConstants.STRENGTH_EXP);
      
      swerveSubsystem.drive(
        new Translation2d(leftX, leftY * (1 - strength) + ySpeed * strength),
        angularVelocityController * (1 - strength) + angleSpeed * strength,
        true
      );
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trenchTarget = Translation2d.kZero;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }


}
