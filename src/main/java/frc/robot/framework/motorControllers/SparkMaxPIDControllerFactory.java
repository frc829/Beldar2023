// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.motorControllers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

/** Add your docs here. */
public class SparkMaxPIDControllerFactory {

    public static SparkMaxPIDController create(
            CANSparkMax canSparkMax,
            double kP,
            double kI,
            double kD,
            double kFF) {
        SparkMaxPIDController sparkMaxPIDController = canSparkMax.getPIDController();
        sparkMaxPIDController.setP(kP);
        sparkMaxPIDController.setI(kI);
        sparkMaxPIDController.setD(kD);
        sparkMaxPIDController.setFF(kFF);

        return sparkMaxPIDController;
    }

}
