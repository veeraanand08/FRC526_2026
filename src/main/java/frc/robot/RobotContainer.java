// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoAlign.Target;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterFallback;
import frc.robot.commands.auton.AutoAlignOnce;
import frc.robot.subsystems.*;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {

  private final edu.wpi.first.wpilibj2.command.button.CommandGenericHID m_keyboard = 
    new edu.wpi.first.wpilibj2.command.button.CommandGenericHID(2); //SIM STUFF DELETE LATER

  private final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  
  private final CommandXboxController operatorController =
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final Vision visionSubsystem;
  private final Shooter shooterSubsystem;
  private final Feeder feederSubsystem;
  public final Intake intakeSubsystem;

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final LoggedDashboardChooser<Command> autoChooser;

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
    switch (currentMode) {
      case REAL:
        visionSubsystem = new Vision(
                swerveSubsystem.getSwerveDrive()::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.CAMERA_0_NAME, swerveSubsystem::getHeading,
                        () -> swerveSubsystem.getRobotVelocity().omegaRadiansPerSecond),
                new VisionIOLimelight(VisionConstants.CAMERA_1_NAME, swerveSubsystem::getHeading,
                        () -> swerveSubsystem.getRobotVelocity().omegaRadiansPerSecond));
        shooterSubsystem = new Shooter(
                new ShooterIOSparkMax(),
                swerveSubsystem::getPose,
                swerveSubsystem::getFieldVelocity);
        feederSubsystem = new Feeder(new FeederIOSparkMax());
        intakeSubsystem = new Intake(new IntakeIOSparkMax());
        break;
      case SIM:
        visionSubsystem = new Vision(
                swerveSubsystem.getSwerveDrive()::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                        VisionConstants.CAMERA_0_NAME,
                        VisionConstants.robotToCamera0,
                        swerveSubsystem::getPose),
                new VisionIOPhotonVisionSim(
                        VisionConstants.CAMERA_1_NAME,
                        VisionConstants.robotToCamera1,
                        swerveSubsystem::getPose));
        shooterSubsystem = new Shooter(
                new ShooterIOSim(),
                swerveSubsystem::getPose,
                swerveSubsystem::getFieldVelocity);
        feederSubsystem = new Feeder(new FeederIOSim());
        intakeSubsystem = new Intake(new IntakeIOSim());
        break;
      default:
        visionSubsystem = new Vision(
                swerveSubsystem.getSwerveDrive()::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {}
        );
        shooterSubsystem = new Shooter(
                new ShooterIO() {},
                swerveSubsystem::getPose,
                swerveSubsystem::getFieldVelocity);
        feederSubsystem = new Feeder(new FeederIO() {});
        intakeSubsystem = new Intake(new IntakeIO() {});
    }

    // Configure the trigger bindings
    configureBindings();

    configureAutoCommands();

    DriverStation.silenceJoystickConnectionWarning(true);

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

    // Set the default auto (do nothing)
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
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
//    Command driveFieldOrientedDirectAngle     = swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
//    Command driveRobotOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
//    Command driveFieldOrientedDirectAngleKeyboard     = swerveSubsystem.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAngularVelocityKeyboard = swerveSubsystem.driveFieldOriented(driveAngularVelocityKeyboard);
//    Command driveSetpointGenKeyboard = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(
//        driveDirectAngleKeyboard);
    Command lockSwerve = Commands.run(swerveSubsystem::lock, swerveSubsystem)
                                 .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
//    Command autoAlignHub = new AutoAlign(swerveSubsystem, m_driverController, Target.HUB);
    Command autoAlign = new AutoAlign(swerveSubsystem, m_driverController, Target.AUTO);
    Command shootAutoSpeed = new ShooterCommand(shooterSubsystem, feederSubsystem, false)
                                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    Command shootDefaultSpeed = new ShooterFallback(shooterSubsystem, feederSubsystem);
    Command AHHH_INDEXER_STUCK_PLEASE_HELP_ME = feederSubsystem.reverse();
    Command holdIntake = intakeSubsystem.intakeCommand();
    Command reverseIntake = intakeSubsystem.reverseIntakeCommand();
    Command agitateIntake = intakeSubsystem.agitateCommand();
    Command resetIntake = intakeSubsystem.resetIntakeCommand();

    if (currentMode == Mode.SIM) {
      swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
      m_keyboard.button(1).whileTrue(holdIntake);
        
      m_keyboard.button(2).whileTrue(reverseIntake);
        
      m_keyboard.button(3).onTrue(intakeSubsystem.agitateCommand());

      m_keyboard.button(4).whileTrue(shootAutoSpeed);
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
//      m_driverController.b().whileTrue(autoAlignHub);
      m_driverController.x().whileTrue(lockSwerve);
      // operator controls (on driver controller)
      m_driverController.leftBumper().whileTrue(holdIntake);
      m_driverController.rightBumper().whileTrue(shootAutoSpeed);
      m_driverController.y().onTrue(agitateIntake);
      m_driverController.povUp().onTrue(resetIntake);
      m_driverController.povDown().whileTrue(AHHH_INDEXER_STUCK_PLEASE_HELP_ME);
      m_driverController.povRight().whileTrue(reverseIntake);
    }
    else
    {
      // Triggers
//      new Trigger(swerveSubsystem::isNearTrench).whileTrue(new TrenchAlign(swerveSubsystem, m_driverController));

      // driver controls
      m_driverController.povLeft().onTrue((Commands.runOnce(swerveSubsystem::zeroGyroWithAlliance)));
      m_driverController.a().whileTrue(autoAlign);
      m_driverController.leftBumper().whileTrue(lockSwerve);
      // operator controls
      operatorController.leftBumper().whileTrue(holdIntake);
      operatorController.rightBumper().whileTrue(shootAutoSpeed);
      operatorController.a().onTrue(agitateIntake);
      operatorController.b().whileTrue(reverseIntake);
      operatorController.rightTrigger(0.7).whileTrue(shootDefaultSpeed);
      operatorController.y().whileTrue(AHHH_INDEXER_STUCK_PLEASE_HELP_ME);
      operatorController.povUp().onTrue(resetIntake);
    }
  }

  private void configureAutoCommands() {
    NamedCommands.registerCommand("toggleIntake", intakeSubsystem.toggleIntakeCommand());
    NamedCommands.registerCommand("Hub Auto Align", new AutoAlignOnce(swerveSubsystem, Target.HUB));
    NamedCommands.registerCommand("Shoot", new ShooterCommand(shooterSubsystem, feederSubsystem, false));
    NamedCommands.registerCommand("Agitate", intakeSubsystem.agitateCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
