// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.types;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class PIDEndAtSetPointCommand extends PIDCommand{

    public PIDEndAtSetPointCommand(PIDController controller, DoubleSupplier measurementSource, double setpoint,
            DoubleConsumer useOutput, Subsystem... requirements) {
        super(controller, measurementSource, setpoint, useOutput, requirements);
    }

    @Override
    public boolean isFinished() {
        return super.m_controller.atSetpoint();
    }
}
