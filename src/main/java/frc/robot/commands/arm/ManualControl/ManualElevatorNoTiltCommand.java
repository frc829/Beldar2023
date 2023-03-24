// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.ManualControl;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ElevatorTiltState;

/** Add your docs here. */
public class ManualElevatorNoTiltCommand extends ManualControlCommand{

    public ManualElevatorNoTiltCommand(Arm arm) {
        super(arm);        
    }

    @Override
    public void initialize() {
        arm.setElevatorTiltState(ElevatorTiltState.NONE);
    }

    @Override
    public void end(boolean interrupted) {
    }

}
