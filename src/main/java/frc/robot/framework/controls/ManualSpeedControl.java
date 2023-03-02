// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.controls;

/** Add your docs here. */
public class ManualSpeedControl {

    private final ControllerAxis controllerAxis;
    private final double maximumSpeed;

    public ManualSpeedControl(
            ControllerAxis controllerAxis,
            double maximumSpeed) {

        this.controllerAxis = controllerAxis;
        this.maximumSpeed = maximumSpeed;
    }

    public double getManualSpeed(){
        return this.controllerAxis.getAxisValue() * maximumSpeed;
    }
}
