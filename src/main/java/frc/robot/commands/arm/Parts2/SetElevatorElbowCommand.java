// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.Parts2;

import frc.robot.commands.arm.ArmDefaultCommand;
import frc.robot.subsystems.Arm;

/** Add your docs here. */
public class SetElevatorElbowCommand extends ArmDefaultCommand {

    public SetElevatorElbowCommand(Arm arm, double elevatorPosition, double elbowPosition) {
        super(arm);
        this.elevatorPosition = elevatorPosition;
        this.elbowPosition = elbowPosition;
    }

    @Override
    public void initialize() {
        this.clawState = arm.getClawState();
        this.elevatorTiltState = arm.getElevatorTiltState();
        this.ledAnimation = arm.getLEDAnimation();
        arm.setElevatorSetpoint(elevatorPosition);
        arm.setElbowSetPoint(elbowPosition);
    }

    @Override
    public boolean isFinished() {
        return arm.elevatorAtSetPoint() && arm.elbowAtSetPoint();
    }
}
