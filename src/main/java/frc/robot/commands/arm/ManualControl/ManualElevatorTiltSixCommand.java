// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.ManualControl;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ElevatorTiltState;

/** Add your docs here. */
public class ManualElevatorTiltSixCommand extends ManualControlCommand{

    public ManualElevatorTiltSixCommand(Arm arm) {
        super(arm);        
    }

    @Override
    public void initialize() {
        arm.setElevatorTiltState(ElevatorTiltState.SIX);
    }
}
