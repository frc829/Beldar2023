// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.ManualControl;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ClawState;

/** Add your docs here. */
public class ManualClawControlCommand extends ManualControlCommand{

    public ManualClawControlCommand(Arm arm) {
        super(arm);        
    }

    @Override
    public void initialize() {
        ClawState clawStateToSet = arm.getClawState() == ClawState.CONE ? ClawState.CUBE : ClawState.CONE;
        arm.setClawState(clawStateToSet);
    }

    @Override
    public void end(boolean interrupted) {
    }

}
