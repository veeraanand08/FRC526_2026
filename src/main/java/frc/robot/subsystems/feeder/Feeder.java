// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private enum IndexerState {
    LEFT_MOTOR_RUNNING,
    RIGHT_MOTOR_RUNNING,
    DISABLED
  }

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private IndexerState indexerState;
  private final Timer timer;

  public Feeder(FeederIO io) {
    this.io = io;

    indexerState = IndexerState.DISABLED;
    timer = new Timer();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

//    if (indexerState != IndexerState.DISABLED) {
//      if (timer.get() > FeederConstants.INDEXER_PERIOD) {
//        setIndexerState(indexerState == IndexerState.LEFT_MOTOR_RUNNING
//            ? IndexerState.RIGHT_MOTOR_RUNNING
//            : IndexerState.LEFT_MOTOR_RUNNING);
//        timer.restart();
//      }
//    }
  }

  private void setIndexerState(IndexerState newState) {
    indexerState = newState;
    switch (newState) {
      case LEFT_MOTOR_RUNNING:
        io.setIndexerLeft(FeederConstants.INDEXER_POWER);
        io.stopIndexerRight();
        break;
      case RIGHT_MOTOR_RUNNING:
        io.stopIndexerLeft();
        io.setIndexerRight(FeederConstants.INDEXER_POWER);
        break;
      case DISABLED:
        io.stopIndexerLeft();
        io.stopIndexerRight();
        timer.stop();
        break;
    }
  }

  public void enableIndexer(boolean reversed) {
    if (reversed) {
      setIndexerState(IndexerState.DISABLED);
      io.setIndexerLeft(-FeederConstants.INDEXER_POWER_REVERSED);
      io.setIndexerRight(-FeederConstants.INDEXER_POWER_REVERSED);
    }
    else {
//      timer.restart();
//      setIndexerState(IndexerState.LEFT_MOTOR_RUNNING);
      io.setIndexerLeft(FeederConstants.INDEXER_POWER);
      io.setIndexerRight(FeederConstants.INDEXER_POWER);
    }
  }

  public void enableKicker() {
    io.setKickerRPM(FeederConstants.KICKER_RPM);
  }

  public void stop() {
    setIndexerState(IndexerState.DISABLED);
    io.stopKicker();
  }

  public Command reverse() {
    return startEnd(() -> {
      enableIndexer(true);
      io.setKickerRPM(FeederConstants.KICKER_RPM_REVERSED);
    }, this::stop);
  }
}