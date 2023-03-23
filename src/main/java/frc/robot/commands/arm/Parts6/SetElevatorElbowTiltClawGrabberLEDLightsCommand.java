// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.Parts6;

import frc.robot.commands.arm.Parts2.SetElevatorElbowCommand;
import frc.robot.framework.lighting.animations.LEDAnimation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ClawState;
import frc.robot.subsystems.Arm.ElevatorTiltState;

/** Add your docs here. */
public class SetElevatorElbowTiltClawGrabberLEDLightsCommand extends SetElevatorElbowCommand {

    public SetElevatorElbowTiltClawGrabberLEDLightsCommand(
        Arm arm, 
        double elevatorPosition, 
        double elbowPosition, 
        ElevatorTiltState elevatorTiltState, 
        ClawState clawState,
        double grabberSpeed,
        LEDAnimation ledAnimation) {
        super(arm, elbowPosition, elbowPosition);
        this.elevatorTiltState = elevatorTiltState;
        this.grabberSpeed = grabberSpeed;
        this.clawState = clawState;
        this.ledAnimation = ledAnimation;
    }

    @Override
    public void initialize() {
        arm.setElevatorSetpoint(elevatorPosition);
        arm.setElbowSetPoint(elbowPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
