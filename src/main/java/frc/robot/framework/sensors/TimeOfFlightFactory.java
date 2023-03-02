// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.sensors;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

/** Add your docs here. */
public class TimeOfFlightFactory {

    public static TimeOfFlight create(
        int sensorId, 
        RangingMode mode,
        int sampleTime){
            
            TimeOfFlight timeOfFlight =  new TimeOfFlight(sensorId);
            timeOfFlight.setRangingMode(mode, sampleTime);
            return timeOfFlight;

        }
}
