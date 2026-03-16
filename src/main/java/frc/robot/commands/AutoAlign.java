// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.RobotUtil;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends Command {
  public enum Target {
    HUB,
    BUMP,
    AUTO,
    NONE {
      @Override
      public String toString() {
        return "N/A";
      }
    }
  }

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerveSubsystem;
  private final CommandXboxController driverController;
  private final Target target;
  // used in shooter subsystem to determine if bot is ready to shoot
  private static Target currentTarget = Target.NONE;
  private static Translation2d virtualTarget = Translation2d.kZero;

  /**
   * Creates a new AutoAlign.
   *
   * @param swerveSubsystem The swerve subsystem used by this command.
   * @param driverController The CommandXboxController object of the driver's controller.
   * @param target The target that this command aligns to.
   */
  public AutoAlign(SwerveSubsystem swerveSubsystem,
    CommandXboxController driverController, Target target)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.driverController = driverController;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (target != Target.AUTO) currentTarget = target;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d robotTranslation = swerveSubsystem.getPose().getTranslation();
    Translation2d targetTranslation = getTargetTranslation(target, robotTranslation);
    virtualTarget = getVirtualTarget(swerveSubsystem.getFieldVelocity(), robotTranslation, targetTranslation);

    Translation2d difference = virtualTarget.minus(robotTranslation);
    Rotation2d angleToTarget = new Rotation2d(difference.getX(), difference.getY());

    double leftY = (RobotUtil.isRedAlliance()) ? driverController.getLeftY() : -driverController.getLeftY();
    double leftX = (RobotUtil.isRedAlliance()) ? driverController.getLeftX() : -driverController.getLeftX();

    swerveSubsystem.driveFieldOriented(swerveSubsystem.rotateToAngle( 
        leftY,
        leftX,
        angleToTarget
    ));
    
  }

  public static Target getTarget(Translation2d robotTranslation, boolean isRedAlliance){
    double robotX = robotTranslation.getX();
    if (!isRedAlliance && robotX < FieldConstants.BLUE_ALLIANCE_BOUNDARY){
        return Target.HUB;
    } else if (isRedAlliance && robotX > FieldConstants.RED_ALLIANCE_BOUNDARY){
      return Target.HUB;
    }

    return Target.BUMP;
  }


  public static Translation2d getTargetTranslation(Target target, Translation2d robotTranslation) {
    boolean isRedAlliance = RobotUtil.isRedAlliance();
    Translation2d targetTranslation;
    if (target == Target.AUTO) {
      target = getTarget(robotTranslation, isRedAlliance);
      currentTarget = target;
    }
    switch (target) {
      case NONE:
      case HUB:
        targetTranslation = isRedAlliance ? FieldConstants.RED_HUB : FieldConstants.BLUE_HUB;
        break;
      case BUMP:
        Translation2d leftBump, rightBump;

        if (isRedAlliance) {
            leftBump = FieldConstants.RED_LEFT_BUMP;
            rightBump = FieldConstants.RED_RIGHT_BUMP;
        } else {
            leftBump = FieldConstants.BLUE_LEFT_BUMP;
            rightBump = FieldConstants.BLUE_RIGHT_BUMP;
        }
        double leftBumpDistance = robotTranslation.getDistance(leftBump);
        double rightBumpDistance = robotTranslation.getDistance(rightBump);

        if (leftBumpDistance < rightBumpDistance){
            targetTranslation = leftBump;
        } else {
            targetTranslation = rightBump;
        }
        break;
      default:
        targetTranslation = robotTranslation;
    }
    return targetTranslation;
  }

  public static Translation2d getVirtualTarget(ChassisSpeeds robotSpeed, Translation2d robotTranslation, Translation2d targetTranslation) {
    Translation2d virtualTargetTranslation = targetTranslation;

    for (int i = 0; i < AutoAlignConstants.MAX_ITERATIONS; i++){
      double distanceToTarget = robotTranslation.getDistance(virtualTargetTranslation);
      double shotTime = AutoAlignConstants.DISTANCE_TO_TIME.get(distanceToTarget);

      double xTranslation = robotSpeed.vxMetersPerSecond * shotTime;
      double yTranslation = robotSpeed.vyMetersPerSecond * shotTime;

      virtualTargetTranslation = targetTranslation.minus(new Translation2d(xTranslation, yTranslation));
    }

    return virtualTargetTranslation;
  }

  public static Translation2d getSavedVirtualTarget() {
    return virtualTarget;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentTarget = Target.NONE;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // cancel command if pose estimator may not be accurate
    return !RobotUtil.isPoseEstimatorReady;
  }

  public static Target getCurrentTarget() {
    return currentTarget;
  }

  public static boolean isActive() {
    return currentTarget != Target.NONE;
  }
}
