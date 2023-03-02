// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public interface ControllerAxis {

    public double getAxisValue();

    public static ControllerAxis getAxisControl(
            CommandXboxController xboxController,
            XboxController.Axis xboxControllerAxis,
            boolean isInverted,
            double xboxControllerAxisDeadband) {
        return new ControllerAxis() {

            @Override
            public double getAxisValue() {
                double rawAxisValue = xboxController.getRawAxis(xboxControllerAxis.value);
                double adjustedAxisValue = MathUtil.applyDeadband(rawAxisValue, xboxControllerAxisDeadband);
                adjustedAxisValue = isInverted ? -adjustedAxisValue : adjustedAxisValue;
                adjustedAxisValue = Math.abs(adjustedAxisValue) > 1.0 ? Math.signum(adjustedAxisValue) : adjustedAxisValue;
                return adjustedAxisValue;
            }
        };
    }

    public static ControllerAxis getAxisControl(
            HalfControllerAxis positiveDirectionAxisControl,
            HalfControllerAxis negativeDirectionAxisControl) {

        return new ControllerAxis() {

            @Override
            public double getAxisValue() {
                return positiveDirectionAxisControl.getAxisValue() - negativeDirectionAxisControl.getAxisValue();
            }
        };
    }

}
