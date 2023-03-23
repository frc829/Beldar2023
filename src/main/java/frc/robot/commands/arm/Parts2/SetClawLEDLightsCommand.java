// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.Parts2;

import frc.robot.commands.arm.Parts1.SetClawCommand;
import frc.robot.framework.lighting.animations.LEDAnimation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ClawState;

/** Add your docs here. */
public class SetClawLEDLightsCommand extends SetClawCommand{

    public SetClawLEDLightsCommand(Arm arm, ClawState clawState, LEDAnimation ledAnimation) {
        super(arm, clawState);
        this.ledAnimation = ledAnimation;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
