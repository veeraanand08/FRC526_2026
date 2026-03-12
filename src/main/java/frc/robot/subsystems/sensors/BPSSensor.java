// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LaserCanConstants;

import java.util.ArrayDeque;
import java.util.Deque;

import org.littletonrobotics.junction.Logger;

public class BPSSensor extends SubsystemBase {

  private final BallSensorIO io;
  private final BallSensorIOInputsAutoLogged inputs = new BallSensorIOInputsAutoLogged();

  private int framesStateChanged;
  private boolean ballInFront;

  private Deque<Double> currentBalls;

  public BPSSensor(BallSensorIO io) {
    this.io = io;
    framesStateChanged = 0;
    ballInFront = false;
    currentBalls = new ArrayDeque<>();

    Logger.recordOutput("BPSSensor/BallInFront", ballInFront);
    Logger.recordOutput("BPSSensor/BPS", getBPS());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("BPSSensor", inputs);

    double currentTimeSecs = Timer.getFPGATimestamp();
    if (inputs.valid){
      if (inputs.distanceMiliMeters < LaserCanConstants.MAXIMUM_BALL_IN_FRONT_DISTANCE != ballInFront) {
        framesStateChanged++;
      } else {
        framesStateChanged = 0;
      }
    }
    

    if (framesStateChanged > 3){
      framesStateChanged = 0;
      ballInFront = !ballInFront;
      if (ballInFront){
        currentBalls.add(currentTimeSecs);
      }
    }

    
    while (!currentBalls.isEmpty() && currentBalls.peek() < currentTimeSecs - 1.0){
      currentBalls.poll();
    }
    
    Logger.recordOutput("BPSSensor/BallInFront", ballInFront);
    Logger.recordOutput("BPSSensor/BPS", getBPS());
  }

  public int getBPS(){
    return currentBalls.size();
  }
}