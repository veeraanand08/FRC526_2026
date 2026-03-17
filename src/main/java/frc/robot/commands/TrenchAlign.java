// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TrenchAlignConstants;
import frc.robot.RobotUtil;
import frc.robot.subsystems.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class TrenchAlign extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final CommandXboxController driverController;
  private Translation2d trenchTarget;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController angleController;


  /**
    * Creates a new TrenchAlign.
    *
    * @param swerveSubsystem The swerve subsystem used by this command.
    * @param driverController The CommandXboxController object of the driver's controller.
    */
  public TrenchAlign(SwerveSubsystem swerveSubsystem,
                     CommandXboxController driverController)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.driverController = driverController;
    this.trenchTarget = null;
    yController = new ProfiledPIDController(TrenchAlignConstants.Y_P, TrenchAlignConstants.Y_I, TrenchAlignConstants.Y_D,
    new TrapezoidProfile.Constraints(
        DriveConstants.MAX_SPEED,
        TrenchAlignConstants.MAX_ACCEL
    )
    );
    angleController = new ProfiledPIDController(TrenchAlignConstants.ANGLE_P, TrenchAlignConstants.ANGLE_I, TrenchAlignConstants.ANGLE_D,
    new TrapezoidProfile.Constraints(
        TrenchAlignConstants.MAX_ANGULAR_SPEED,
        TrenchAlignConstants.MAX_ANGULAR_ACCEL
    )
    );
    
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trenchTarget = findNearestTrench();
    yController.reset(swerveSubsystem.getPose().getY());
    angleController.reset(swerveSubsystem.getHeading().getRadians());
    Logger.recordOutput("TrenchAlign/Trench Align Running", true);
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

    leftX *= DriveConstants.MAX_SPEED;
    leftY *= DriveConstants.MAX_SPEED;

    if (trenchTarget != null) {
      double ySpeed = yController.calculate(swerveSubsystem.getPose().getY(), trenchTarget.getY());

      double angleSpeed = angleController.calculate(swerveSubsystem.getHeading().getRadians(), Math.toRadians(TrenchAlignConstants.TRENCH_ROTATION_SETPOINT));
      
      double distance = Math.abs(swerveSubsystem.getPose().getX() - trenchTarget.getX());

      Logger.recordOutput("TrenchAlign/Desired Angle", TrenchAlignConstants.TRENCH_ROTATION_SETPOINT);

      Logger.recordOutput("TrenchAlign/Desired Y", trenchTarget.getY());

 
      double fullStrengthDist = 0.2;

      double adjustedDistance = Math.max(0.0, distance - fullStrengthDist);

      double adjustedThreshold = TrenchAlignConstants.TRENCH_ALIGNMENT_THRESHOLD - fullStrengthDist;

      double normalizedDist = MathUtil.clamp(1.0 - (adjustedDistance / Math.max(0.001, adjustedThreshold)), 0.0, 1.0);
      double strength = Math.pow(normalizedDist, TrenchAlignConstants.STRENGTH_EXP);

      Logger.recordOutput("TrenchAlign/strength", strength);
      swerveSubsystem.drive(
        new Translation2d(leftY, leftX * (1.0 - strength) + ySpeed * strength),
        rightX * (1.0 - strength) + angleSpeed * strength,
        true
      );
    }
  }

//  public boolean isNearTrench() {
//    return findNearestTrench() != null;
//  }

  public Translation2d findNearestTrench() {
    Translation2d robotPose = swerveSubsystem.getPose().getTranslation();
    Translation2d leftTrench, rightTrench;
    if (RobotUtil.isRedAlliance()) {
      leftTrench = FieldConstants.RED_LEFT_TRENCH;
      rightTrench = FieldConstants.RED_RIGHT_TRENCH;
    } else {
      leftTrench = FieldConstants.BLUE_LEFT_TRENCH;
      rightTrench = FieldConstants.BLUE_RIGHT_TRENCH;
    }
    double leftTrenchDist = robotPose.getDistance(leftTrench);
    double leftTrenchYDist = Math.abs( robotPose.getY() - leftTrench.getY() );
    if (leftTrenchDist < TrenchAlignConstants.TRENCH_ALIGNMENT_THRESHOLD && leftTrenchYDist < TrenchAlignConstants.TRENCH_ALIGNMENT_Y_THRESHOLD){
      return leftTrench;
    }
    double rightTrenchDist = robotPose.getDistance(rightTrench);
    double rightTrenchYDist = Math.abs( robotPose.getY() - rightTrench.getY() );
    if (rightTrenchDist < TrenchAlignConstants.TRENCH_ALIGNMENT_THRESHOLD && rightTrenchYDist < TrenchAlignConstants.TRENCH_ALIGNMENT_Y_THRESHOLD){
      return rightTrench;
    }

    return null;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trenchTarget = null;
    Logger.recordOutput("TrenchAlign/Trench Align Running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}