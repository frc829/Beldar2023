// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.sensors;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.simulation.Simulator;

/** Add your docs here. */
public class CANCoderFactory {

    public static WPI_CANCoder create(
            int deviceId,
            String canbus,
            Motor motor,
            double motorToMechConversion) {
        WPI_CANCoder cancoder = new WPI_CANCoder(deviceId, canbus);
        cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        if (RobotBase.isSimulation()) {
            Simulator.getInstance().addCANCoder(cancoder, motor, motorToMechConversion);
        }

        return cancoder;
    }
}
