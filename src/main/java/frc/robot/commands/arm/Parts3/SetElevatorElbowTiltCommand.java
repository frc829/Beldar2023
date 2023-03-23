// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.Parts3;

import frc.robot.commands.arm.Parts2.SetElevatorElbowCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ElevatorTiltState;

/** Add your docs here. */
public class SetElevatorElbowTiltCommand extends SetElevatorElbowCommand {

    public SetElevatorElbowTiltCommand(
        Arm arm, 
        double elevatorPosition, 
        double elbowPosition, 
        ElevatorTiltState elevatorTiltState) {
        super(arm, elbowPosition, elbowPosition);
        this.elevatorTiltState = elevatorTiltState;
    }

    @Override
    public void initialize() {
        this.clawState = arm.getClawState();
        this.ledAnimation = arm.getLEDAnimation();
        arm.setElevatorSetpoint(elevatorPosition);
        arm.setElbowSetPoint(elbowPosition);
    }
}
