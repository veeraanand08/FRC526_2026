// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveDrive;

/** An example command that uses an example subsystem. */
public class AutoAlign extends Command {
  public enum Target {
    HUB,
    BUMP
  }

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveDrive swerveDrive;
  private final VisionSubsystem visionSubsystem;
  private final CommandXboxController driverController;
  private final Target target;
  // used in shooter subsystem to determine if bot is ready to shoot
  private static Target currentTarget;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlign(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem,
    CommandXboxController driverController, Target target) {
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
    // ensure at least one vision measurement has been added
    if (!visionSubsystem.isPoseEstimatorReady() && !RobotBase.isSimulation()) 
      return;
    Translation2d robotTranslation = swerveDrive.getPose().getTranslation();
    Translation2d targetTranslation;
    switch (target) {
      case HUB:
        targetTranslation = swerveSubsystem.isRedAlliance() ? FieldConstants.RED_HUB : FieldConstants.BLUE_HUB;
        break;
      case BUMP:
        Translation2d leftBump, rightBump;

        if (swerveSubsystem.isRedAlliance()){
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
    Translation2d virtualTarget = getTargetTranslation(robotTranslation, targetTranslation);

    Translation2d difference = virtualTarget.minus(robotTranslation);
    Rotation2d angleToTarget = new Rotation2d(difference.getX(), difference.getY());

    double leftY = (swerveSubsystem.isRedAlliance()) ? driverController.getLeftY() : -driverController.getLeftY();
    double leftX = (swerveSubsystem.isRedAlliance()) ? driverController.getLeftX() : -driverController.getLeftX();

    swerveSubsystem.driveFieldOriented(swerveSubsystem.rotateToAngle( 
        leftY,
        leftX,
        angleToTarget
    ));
  }

  private Translation2d getTargetTranslation(Translation2d robotTranslation, Translation2d targetTranslation) {    
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
      currentTarget = null;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static boolean isActive() {
    return currentTarget != null;
  }
}
