// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.Parts4;

import frc.robot.commands.arm.Parts2.SetElevatorElbowCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ElevatorTiltState;

/** Add your docs here. */
public class SetElevatorElbowTiltGrabberCommand extends SetElevatorElbowCommand {

    public SetElevatorElbowTiltGrabberCommand(
        Arm arm, 
        double elevatorPosition, 
        double elbowPosition, 
        ElevatorTiltState elevatorTiltState,
        double grabberSpeedRPM) {
        super(arm, elbowPosition, elbowPosition);
        this.grabberSpeed = grabberSpeedRPM;
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
