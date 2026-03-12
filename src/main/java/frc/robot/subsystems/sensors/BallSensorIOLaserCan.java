package frc.robot.subsystems.sensors;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import frc.robot.Constants.LaserCanConstants;

public class BallSensorIOLaserCan implements BallSensorIO {

    private LaserCan laserCan;

    public BallSensorIOLaserCan(){
        laserCan = new LaserCan(LaserCanConstants.KICKER_LASERCAN);
        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed! " + e);
        }
    }

    public void updateInputs(BallSensorIOInputs inputs){
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            inputs.distanceMiliMeters = measurement.distance_mm;
            inputs.valid = true;
        } else {
            inputs.valid = false;
        }
        
    }
}
