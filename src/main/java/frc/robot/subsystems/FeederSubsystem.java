// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax indexerLeftMotor; // leader
  private final SparkMax indexerRightMotor; // follower
  private final SparkMax kickerMotor;

  private final RelativeEncoder kickerEncoder;
  private final SparkClosedLoopController kickerPid;

  public FeederSubsystem() {
    indexerLeftMotor = new SparkMax(FeederConstants.LEFT_INDEXER_MOTOR, MotorType.kBrushless);
    indexerRightMotor = new SparkMax(FeederConstants.RIGHT_INDEXER_MOTOR, MotorType.kBrushless);
    kickerMotor = new SparkMax(FeederConstants.KICKER_MOTOR, MotorType.kBrushless);

    SparkMaxConfig indexerLeftMotorConfig = new SparkMaxConfig();
    SparkMaxConfig indexerRightMotorConfig = new SparkMaxConfig();
    SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();

    indexerLeftMotorConfig.inverted(FeederConstants.LEFT_INDEXER_MOTOR_REVERSED);
    indexerLeftMotorConfig.idleMode(IdleMode.kCoast);
    indexerLeftMotorConfig.smartCurrentLimit(FeederConstants.INDEXER_CURRENT_LIMIT);

    indexerRightMotorConfig.inverted(FeederConstants.RIGHT_INDEXER_MOTOR_REVERSED);
    indexerRightMotorConfig.idleMode(IdleMode.kCoast);
    indexerRightMotorConfig.smartCurrentLimit(FeederConstants.INDEXER_CURRENT_LIMIT);

    kickerMotorConfig.inverted(FeederConstants.KICKER_MOTOR_REVERSED);
    kickerMotorConfig.idleMode(IdleMode.kCoast);
    kickerMotorConfig.smartCurrentLimit(FeederConstants.KICKER_CURRENT_LIMIT);
    kickerMotorConfig.closedLoop.pid(FeederConstants.KICKER_P, FeederConstants.KICKER_I, FeederConstants.KICKER_D)
                                .feedForward.kV(FeederConstants.KICKER_FF);

    indexerLeftMotor.configure(indexerLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    indexerRightMotor.configure(indexerRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kickerMotor.configure(kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    kickerPid = kickerMotor.getClosedLoopController();
    kickerEncoder = kickerMotor.getEncoder();

    SmartDashboard.setDefaultNumber("Kicker/Kicker RPM", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Kicker/Kicker RPM", kickerEncoder.getVelocity());
  }

  public void enableIndexer(boolean counterRotate){
    indexerLeftMotor.set(FeederConstants.INDEXER_POWER);
    if (counterRotate) indexerRightMotor.set(-FeederConstants.INDEXER_POWER);
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
