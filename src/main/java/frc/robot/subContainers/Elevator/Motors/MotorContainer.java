// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subContainers.Elevator.Motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.framework.motorControllers.MotorController;
import frc.robot.framework.motorControllers.SparkMaxPIDControllerFactory;
import frc.robot.framework.motorEncoders.MotorEncoder;
import frc.robot.framework.motorEncoders.RelativeEncoderFactory;
import frc.robot.framework.motors.SparkMaxFactory;

/** Add your docs here. */
public class MotorContainer {

    public final MotorEncoder motorEncoder;
    public final MotorController motorController;

    public MotorContainer() {

        CANSparkMax canSparkMax = SparkMaxFactory.create(
                Constants.deviceId,
                Constants.sparkMaxCompatibleType);

        RelativeEncoder relativeEncoder = RelativeEncoderFactory.create(
                canSparkMax,
                Constants.sparkMaxCompatibleType);

        SparkMaxPIDController sparkMaxPIDController = SparkMaxPIDControllerFactory.create(
                canSparkMax,
                Constants.kP,
                Constants.kI,
                Constants.kD,
                Constants.kFF);

        this.motorEncoder = MotorEncoder.create(relativeEncoder);
        this.motorController = MotorController.create(sparkMaxPIDController);

    }
}
