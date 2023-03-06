// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.motorEncoders;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import frc.robot.framework.motors.SparkMaxCompatibleTypes;

/** Add your docs here. */
public class RelativeEncoderFactory {

    public static RelativeEncoder create(
            CANSparkMax canSparkMax,
            SparkMaxCompatibleTypes sparkMaxCompatibleType) {
        return canSparkMax.getEncoder(Type.kHallSensor, 42);
    }
}
