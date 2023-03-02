// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.sensors;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.simulation.Simulator;

/** Add your docs here. */
public class WPI_TimeOfFlightFactory {

    public static WPI_TimeOfFlight create(
            TimeOfFlight timeOfFlight,
            int sensorId) {
        return WPI_TimeOfFlightFactory.create(
                timeOfFlight,
                sensorId,
                null,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN);
    }

    public static WPI_TimeOfFlight create(
            TimeOfFlight timeOfFlight,
            int sensorId,
            Motor motor,
            double motorToMechConversion,
            double rotationToLinearConversion,
            double minimumPositionMeters,
            double maximumPositionMeters) {
        if (RobotBase.isSimulation()) {
            SimDevice simTimeOfFlight = SimDevice.create("PWF:TimeOfFlight", sensorId);
            SimDouble simRange = simTimeOfFlight.createDouble("Range (mm)", Direction.kInput, sensorId);

            Simulator.getInstance().addPWFTimeOfFlightSensor(
                    sensorId,
                    motor,
                    motorToMechConversion,
                    rotationToLinearConversion,
                    minimumPositionMeters,
                    maximumPositionMeters);

            return new WPI_TimeOfFlight() {

                @Override
                public double getRange() {
                    return simRange.get();
                }

                @Override
                public boolean isRangeValid() {
                    return false;
                }
            };
        } else {

            return new WPI_TimeOfFlight() {

                @Override
                public double getRange() {
                    SmartDashboard.putNumber("TOF[" + sensorId + "]", timeOfFlight.getRange());
                    return timeOfFlight.getRange();
                }

                @Override
                public boolean isRangeValid() {
                    return timeOfFlight.isRangeValid();
                }


            };
        }
    }

    public interface WPI_TimeOfFlight {

        public double getRange();

        public boolean isRangeValid();
    }
}
