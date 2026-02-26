// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoAlign.Target;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import swervelib.SwerveInputStream;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  private final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  
  private final CommandXboxController operatorController =
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final Vision visionSubsystem = new Vision(
          (visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs) ->
                  swerveSubsystem.getSwerveDrive().addVisionMeasurement(
//                          new Pose2d(visionRobotPoseMeters.getTranslation(), swerveSubsystem.getHeading()),
//                          timestampSeconds),
                          visionRobotPoseMeters,
                          timestampSeconds,
                          visionMeasurementStdDevs),
          new VisionIOLimelight(VisionConstants.CAMERA_0_NAME, swerveSubsystem::getHeading,
                  () -> swerveSubsystem.getRobotVelocity().omegaRadiansPerSecond),
          new VisionIOLimelight(VisionConstants.CAMERA_1_NAME, swerveSubsystem::getHeading,
                  () -> swerveSubsystem.getRobotVelocity().omegaRadiansPerSecond));
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(swerveSubsystem::getPose);
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                () -> -m_driverController.getLeftY(),
                                                                () -> -m_driverController.getLeftX())
                                                            .withControllerRotationAxis(() -> -m_driverController.getRightX())
                                                            .deadband(ControllerConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);

  /**0
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(ControllerConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle     = swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard     = swerveSubsystem.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAngularVelocityKeyboard = swerveSubsystem.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
    Command autoAlignHub = new AutoAlign(swerveSubsystem, m_driverController, Target.HUB);
    Command autoAlign = new AutoAlign(swerveSubsystem, m_driverController, Target.AUTO);
    Command shootAutoSpeed = new ShooterCommand(shooterSubsystem, feederSubsystem, false)
                                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    Command AHHH_INDEXER_STUCK_PLEASE_HELP_ME = feederSubsystem.reverse();
    Command toggleIntake = intakeSubsystem.toggleIntakeCommand();
    Command reverseIntake = intakeSubsystem.reverseIntakeCommand();
    Command agitateIntake = intakeSubsystem.agitateCommand();
    Command resetIntake = intakeSubsystem.resetIntakeCommand();

    NamedCommands.registerCommand("Hub Auto Align", autoAlignHub);
    NamedCommands.registerCommand("Shoot", shootAutoSpeed);

    if (RobotBase.isSimulation()) {
      swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
    }
    else {
      swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    if (DriverStation.isTest())
    {
      swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!

      // driver controls
      m_driverController.povLeft().onTrue((Commands.runOnce(swerveSubsystem::zeroGyroWithAlliance)));
      m_driverController.a().whileTrue(autoAlign);
      m_driverController.b().whileTrue(autoAlignHub);
      m_driverController.x().whileTrue(Commands.run(swerveSubsystem::lock, swerveSubsystem)
                                               .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
      // operator controls (on driver controller)
      m_driverController.leftBumper().onTrue(toggleIntake);
      m_driverController.rightBumper().whileTrue(shootAutoSpeed);
      m_driverController.y().onTrue(agitateIntake);
      m_driverController.povUp().onTrue(resetIntake);
      m_driverController.povDown().whileTrue(AHHH_INDEXER_STUCK_PLEASE_HELP_ME);
      m_driverController.povRight().whileTrue(reverseIntake);
    }
    else
    {
      // driver controls
      m_driverController.povLeft().onTrue((Commands.runOnce(swerveSubsystem::zeroGyroWithAlliance)));
      m_driverController.a().whileTrue(autoAlign);
      m_driverController.leftBumper().whileTrue(Commands.run(swerveSubsystem::lock, swerveSubsystem)
                                                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
      // operator controls
      operatorController.leftBumper().onTrue(toggleIntake);
      operatorController.rightBumper().whileTrue(shootAutoSpeed);
      operatorController.y().whileTrue(AHHH_INDEXER_STUCK_PLEASE_HELP_ME);
      operatorController.a().onTrue(agitateIntake);
      operatorController.b().whileTrue(reverseIntake);
      operatorController.povUp().onTrue(resetIntake);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public IntakeSubsystem getIntakeSubsystem() { return intakeSubsystem; }
}
