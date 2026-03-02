// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TrenchAlignmentConstants;
import frc.robot.RobotUtil;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.math.SwerveMath;
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
    SmartDashboard.putBoolean("TrenchAlign/Trench Align Running", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = (RobotUtil.isRedAlliance()) ? driverController.getLeftY() : -driverController.getLeftY();
    double leftX = (RobotUtil.isRedAlliance()) ? driverController.getLeftX() : -driverController.getLeftX();
    double rightX = (RobotUtil.isRedAlliance()) ? driverController.getRightX() : -driverController.getRightX();

    leftY = MathUtil.applyDeadband(leftY, ControllerConstants.DEADBAND);
    leftX = MathUtil.applyDeadband(leftX, ControllerConstants.DEADBAND);
    rightX = MathUtil.applyDeadband(rightX, ControllerConstants.DEADBAND);

    leftX *= DrivebaseConstants.MAX_SPEED;
    leftY *= DrivebaseConstants.MAX_SPEED;

    if (!trenchTarget.equals(Translation2d.kZero)){
      double ySpeed = yController.calculate(swerveSubsystem.getPose().getY(), trenchTarget.getY());

      double angleSpeed = angleController.calculate(swerveSubsystem.getHeading().getRadians(), Math.toRadians(TrenchAlignmentConstants.TRENCH_ROTATION_SETPOINT));
      
      double distance = Math.abs(swerveSubsystem.getPose().getX() - trenchTarget.getX());

      SmartDashboard.putNumber("TrenchAlign/Desired Angle", TrenchAlignmentConstants.TRENCH_ROTATION_SETPOINT);

      SmartDashboard.putNumber("TrenchAlign/Desired Y", trenchTarget.getY());

 
      double fullStrengthDist = 0.2;

      double adjustedDistance = Math.max(0.0, distance - fullStrengthDist);

      double adjustedThreshold = TrenchAlignmentConstants.TRENCH_ALIGNMENT_THRESHOLD - fullStrengthDist;

      double normalizedDist = MathUtil.clamp(1.0 - (adjustedDistance / Math.max(0.001, adjustedThreshold)), 0.0, 1.0);
      double strength = Math.pow(normalizedDist, TrenchAlignmentConstants.STRENGTH_EXP);


      SmartDashboard.putNumber("TrenchAlign/strength", strength);
      swerveSubsystem.drive(
        new Translation2d(leftY, leftX * (1.0 - strength) + ySpeed * strength),
        rightX * (1.0 - strength) + angleSpeed * strength,
        true
      );
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trenchTarget = Translation2d.kZero;
    SmartDashboard.putBoolean("TrenchAlign/Trench Align Running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }


}
