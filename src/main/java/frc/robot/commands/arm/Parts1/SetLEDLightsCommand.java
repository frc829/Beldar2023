// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.Parts1;

import frc.robot.commands.arm.ArmDefaultCommand;
import frc.robot.framework.lighting.animations.LEDAnimation;
import frc.robot.subsystems.Arm;

/** Add your docs here. */
public class SetLEDLightsCommand extends ArmDefaultCommand {

    public SetLEDLightsCommand(Arm arm, LEDAnimation ledAnimation) {
        super(arm);
        this.ledAnimation = ledAnimation;
    }

    @Override
    public void initialize() {

        this.elevatorPosition = arm.getElevatorPosition();
        this.elbowPosition = arm.getElbowPosition();
        this.elevatorTiltState = arm.getElevatorTiltState();
        this.clawState = arm.getClawState();
        arm.setElevatorSetpoint(elevatorPosition);
        arm.setElbowSetPoint(elbowPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
