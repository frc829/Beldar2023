// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.switches;

import frc.robot.framework.sensors.LinearPositionSensor;

/** Add your docs here. */
public interface LimitSwitch {

    public boolean isOn();

    public static LimitSwitch create(
            LinearPositionSensor linearPositionSensor,
            double minimumDistance,
            double maximumDistance) {

        return new LimitSwitch() {

            @Override
            public boolean isOn() {
                double position = linearPositionSensor.getPosition();

                return position >= minimumDistance && position <= maximumDistance && linearPositionSensor.isRangeValid();
            }
        };
    }
}
