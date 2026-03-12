// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

import java.util.ArrayDeque;
import java.util.Deque;

import org.littletonrobotics.junction.Logger;

public class BallSensor extends SubsystemBase {
  private final BallSensorIO io;
  private final BallSensorIOInputsAutoLogged inputs = new BallSensorIOInputsAutoLogged();

  private int framesChanged;
  private boolean ballInFront;

  private final Deque<Double> currentBalls;

  public BallSensor(BallSensorIO io) {
    this.io = io;
    framesChanged = 0;
    ballInFront = false;
    currentBalls = new ArrayDeque<>();

    Logger.recordOutput("BallSensor/Ball Detected", ballInFront);
    Logger.recordOutput("BallSensor/BPS", getBPS());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("BallSensor", inputs);

    double currentTimeSecs = Timer.getTimestamp();
    if (inputs.valid) {
      if (inputs.distanceMillimeters < FeederConstants.MAXIMUM_BALL_IN_FRONT_DISTANCE != ballInFront) {
        framesChanged++;
      }
      else {
        framesChanged = 0;
      }
    }
    

    if (framesChanged > 3){
      framesChanged = 0;
      ballInFront = !ballInFront;
      if (ballInFront) {
        currentBalls.add(currentTimeSecs);
      }
    }

    
    while (!currentBalls.isEmpty() && currentBalls.peek() < currentTimeSecs - 1.0) {
      currentBalls.poll();
    }
    
    Logger.recordOutput("BallSensor/Ball Detected", ballInFront);
    Logger.recordOutput("BallSensor/BPS", getBPS());
  }

  public int getBPS() {
    return currentBalls.size();
  }

  public boolean isBPSLow() {
    return getBPS() <= FeederConstants.LOW_BPS;
  }
}