// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.AutoAlign.Target;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import limelight.networktables.LimelightSettings.LEDMode;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(swerveSubsystem.getSwerveDrive(), visionSubsystem);
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driverController::getRightX)
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
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

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
    Command autoAlignHub = new AutoAlign(swerveSubsystem, visionSubsystem, m_driverController, Target.HUB)
                                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    Command autoAlignBump = new AutoAlign(swerveSubsystem, visionSubsystem, m_driverController, Target.BUMP);
    Command shootAutoSpeed = new ShooterCommand(shooterSubsystem, feederSubsystem, intakeSubsystem, false);
    Command AHHH_INDEXER_STUCK_PLEASE_HELP_ME = new ShooterCommand(shooterSubsystem, feederSubsystem, intakeSubsystem, true);
    Command holdIntake = new IntakeCommand(intakeSubsystem);
    Command toggleIntake = intakeSubsystem.toggleIntakeCommand();
    Command resetIntake = intakeSubsystem.resetIntakeCommand();

    NamedCommands.registerCommand("Hub Auto Align", autoAlignHub);
    NamedCommands.registerCommand("Shoot", shootAutoSpeed);

    if (RobotBase.isSimulation()) {
      swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
    }
    else {
      swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    // driver controls
    m_driverController.y().onTrue((Commands.runOnce(swerveSubsystem::zeroGyroWithAlliance)));
    m_driverController.a().whileTrue(autoAlignHub);
    m_driverController.b().whileTrue(autoAlignBump);
    m_driverController.leftBumper().whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());

    // operator controls
    if (DriverStation.isTest())
    {
      swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!

      m_driverController.povDown().onTrue(toggleIntake);
      m_driverController.povUp().onTrue(resetIntake);
      m_driverController.rightBumper().whileTrue(shootAutoSpeed);
      m_driverController.x().whileTrue(AHHH_INDEXER_STUCK_PLEASE_HELP_ME);
      m_driverController.povLeft().onTrue(Commands.runOnce(visionSubsystem::toggleLED));
    }
    else
    {
      // operatorController.povDown().onTrue(toggleIntake);
      // operatorController.leftBumper().whileTrue(holdIntake);
      operatorController.leftBumper().onTrue(toggleIntake);
      operatorController.povUp().onTrue(resetIntake);
      operatorController.rightBumper().whileTrue(shootAutoSpeed);
      operatorController.y().whileTrue(AHHH_INDEXER_STUCK_PLEASE_HELP_ME);
      operatorController.povLeft().onTrue(Commands.runOnce(visionSubsystem::toggleLED));
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
}
