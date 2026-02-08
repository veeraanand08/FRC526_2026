// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax indexerLeftMotor; // leader
  private final SparkMax indexerRightMotor; // follower
  private final SparkMax kickerMotor;
  
  private final SparkMaxConfig indexerLeftMotorConfig;
  private final SparkMaxConfig indexerRightMotorConfig;
  private final SparkMaxConfig kickerMotorConfig;

  private final SparkClosedLoopController kickerPid;

  private final RelativeEncoder kickerEncoder;
  private double kickerRPM;

  public FeederSubsystem() {
    indexerLeftMotor = new SparkMax(FeederConstants.LEFT_INDEXER_MOTOR, MotorType.kBrushless);
    indexerRightMotor = new SparkMax(FeederConstants.RIGHT_INDEXER_MOTOR, MotorType.kBrushless);
    kickerMotor = new SparkMax(FeederConstants.KICKER_MOTOR, MotorType.kBrushless);

    indexerLeftMotorConfig = new SparkMaxConfig();
    indexerRightMotorConfig = new SparkMaxConfig();
    kickerMotorConfig = new SparkMaxConfig();

    indexerLeftMotorConfig.inverted(FeederConstants.LEFT_INDEXER_MOTOR_REVERSED);
    indexerRightMotorConfig.inverted(FeederConstants.RIGHT_INDEXER_MOTOR_REVERSED);
    kickerMotorConfig.inverted(FeederConstants.KICKER_MOTOR_REVERSED);

    indexerLeftMotorConfig.idleMode(IdleMode.kCoast);
    indexerRightMotorConfig.idleMode(IdleMode.kCoast);
    kickerMotorConfig.idleMode(IdleMode.kCoast);

    kickerMotorConfig.closedLoop.pid(FeederConstants.KICKER_P, FeederConstants.KICKER_I, FeederConstants.KICKER_D);

    indexerLeftMotor.configure(indexerLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    indexerRightMotor.configure(indexerRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kickerMotor.configure(kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    kickerPid = kickerMotor.getClosedLoopController();

    kickerEncoder = kickerMotor.getEncoder();
  }

  @Override
  public void periodic() {
    kickerRPM = kickerEncoder.getVelocity();
    SmartDashboard.putNumber("Kicker/Kicker RPM", kickerRPM);
  }

  public void enableIndexer(boolean reversed){
    indexerLeftMotor.set(FeederConstants.INDEXER_POWER);
    if (reversed) indexerRightMotor.set(-FeederConstants.INDEXER_POWER);
    else indexerRightMotor.set(FeederConstants.INDEXER_POWER);
  }

  public void enableKicker() {
    kickerPid.setSetpoint(FeederConstants.KICKER_RPM, ControlType.kVelocity);
  }

  public void stop() {
    indexerLeftMotor.set(0);
    indexerRightMotor.set(0);
    kickerMotor.set(0);
  }
}
