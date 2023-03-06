// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public abstract class SparkMaxFactory {

    public static CANSparkMax create(
            int deviceId,
            SparkMaxCompatibleTypes sparkMaxCompatibleType) {

        CANSparkMax canSparkMax = new CANSparkMax(deviceId, MotorType.kBrushless);

        if (sparkMaxCompatibleType == SparkMaxCompatibleTypes.NEO) {
            canSparkMax.setSmartCurrentLimit(40);
        } else {
            canSparkMax.setSmartCurrentLimit(20);
        }

        return canSparkMax;

    }
}
