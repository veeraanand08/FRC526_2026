// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotUtil;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveDrive;

public class AutoAlign extends Command {
  public enum Target {
    HUB,
    BUMP,
    NONE {
      @Override
      public String toString() {
        return "N/A";
      }
    }
  }

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveDrive swerveDrive;
  private final VisionSubsystem visionSubsystem;
  private final CommandXboxController driverController;
  private final Target target;
  // used in shooter subsystem to determine if bot is ready to shoot
  private static Target currentTarget = Target.NONE;

  /**
   * Creates a new AutoAlign.
   *
   * @param swerveSubsystem The swerve subsystem used by this command.
   * @param visionSubsystem The vision subsystem used by this command.
   * @param driverController The CommandXboxController object of the driver's controller.
   * @param target The target that this command aligns to.
   */
  public AutoAlign(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem,
    CommandXboxController driverController, Target target)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.getSwerveDrive();
    this.visionSubsystem = visionSubsystem;
    this.driverController = driverController;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentTarget = target;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d robotTranslation = swerveDrive.getPose().getTranslation();
    Translation2d targetTranslation = getTargetTranslation(target, robotTranslation);
    Translation2d virtualTarget = getVirtualTarget(robotTranslation, targetTranslation);

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

  public static Translation2d getTargetTranslation(Target target, Translation2d robotTranslation) {
    Translation2d targetTranslation;
    switch (target) {
      case HUB:
        targetTranslation = RobotUtil.isRedAlliance() ? FieldConstants.RED_HUB : FieldConstants.BLUE_HUB;
        break;
      case BUMP:
        Translation2d leftBump, rightBump;

        if (RobotUtil.isRedAlliance()){
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
    SmartDashboard.putString("AutoAlign/Target Pose", targetTranslation.toString());
    return targetTranslation;
  }

  private Translation2d getVirtualTarget(Translation2d robotTranslation, Translation2d targetTranslation) {    
    ChassisSpeeds robotSpeed = swerveDrive.getFieldVelocity();
    Translation2d virtualTargetTranslation = targetTranslation;

    for (int i = 0; i < ShooterConstants.MAX_ITERATIONS; i++){
      double distanceToTarget = robotTranslation.getDistance(virtualTargetTranslation);
      double shotTime = ShooterConstants.DISTANCE_TO_TIME.get(distanceToTarget);

      double xTranslation = robotSpeed.vxMetersPerSecond * shotTime;
      double yTranslation = robotSpeed.vyMetersPerSecond * shotTime;

      virtualTargetTranslation = targetTranslation.minus(new Translation2d(xTranslation, yTranslation));
    }

    return virtualTargetTranslation;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (currentTarget == target)
      currentTarget = Target.NONE;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // cancel command if pose estimator may not be accurate
    return !visionSubsystem.isPoseEstimatorReady();
  }

  public static Target getCurrentTarget() {
    return currentTarget;
  }

  public static boolean isActive() {
    return currentTarget != Target.NONE;
  }
}
