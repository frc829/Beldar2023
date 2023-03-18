// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.controls;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

/** Add your docs here. */
public class PIDControllerFactory {

    public static PIDController create(
            double kP,
            double kI,
            double kD,
            double tolerance) {
        PIDController pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        return pidController;
    }

    public static PIDController create(
            double kP,
            double kI,
            double kD,
            double tolerance,
            double minContinuousPosition,
            double maxContinuousPosition) {
        PIDController pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        pidController.enableContinuousInput(minContinuousPosition, maxContinuousPosition);
        return pidController;
    }

    public static ProfiledPIDController createProfiled(
        double kP,
        double kI,
        double kD,
        double tolerance,
        double minContinuousPosition,
        double maxContinuousPosition) {
    PIDController pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(tolerance);
    pidController.enableContinuousInput(minContinuousPosition, maxContinuousPosition);
    return pidController;
}

}
