// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
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

public class FeederSubsystem extends SubsystemBase {
  private enum IndexerState {
    LEFT_MOTOR_RUNNING,
    RIGHT_MOTOR_RUNNING,
    DISABLED
  }

  private IndexerState indexerState;
  private final Timer timer;

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

    indexerState = IndexerState.DISABLED;
    timer = new Timer();

    SmartDashboard.setDefaultNumber("Kicker/Kicker RPM", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Kicker/Kicker RPM", kickerEncoder.getVelocity());

    if (indexerState != IndexerState.DISABLED){
      if (timer.get() > FeederConstants.INDEXER_PERIOD) {
        setIndexerState(indexerState == IndexerState.LEFT_MOTOR_RUNNING
            ? IndexerState.RIGHT_MOTOR_RUNNING
            : IndexerState.LEFT_MOTOR_RUNNING);
        timer.restart();
      }
    }
  }

  private void setIndexerState(IndexerState newState) {
    indexerState = newState;
    switch (newState) {
      case LEFT_MOTOR_RUNNING:
        indexerLeftMotor.set(FeederConstants.INDEXER_POWER);
        indexerRightMotor.set(0);
        break;
      case RIGHT_MOTOR_RUNNING:
        indexerLeftMotor.set(0);
        indexerRightMotor.set(FeederConstants.INDEXER_POWER);
        break;
      case DISABLED:
        indexerLeftMotor.set(0);
        indexerRightMotor.set(0);
        timer.stop();
        break;
    }
  }

  public void enableIndexer(boolean reversed) {
    if (reversed) {
      setIndexerState(IndexerState.DISABLED);
      indexerRightMotor.set(-FeederConstants.INDEXER_POWER);
      indexerLeftMotor.set(FeederConstants.INDEXER_POWER);
    }
    else {
      timer.restart();
      setIndexerState(IndexerState.LEFT_MOTOR_RUNNING);
    }
  }

  public void enableKicker() {
    kickerPid.setSetpoint(FeederConstants.KICKER_RPM, ControlType.kVelocity);
  }

  public void stop() {
    setIndexerState(IndexerState.DISABLED);
    kickerMotor.set(0);
  }

  public Command reverse() {
    return startEnd(() -> {
      enableIndexer(true);
      kickerPid.setSetpoint(FeederConstants.KICKER_RPM_REVERSED, ControlType.kVelocity);
    }, this::stop);
  }
}