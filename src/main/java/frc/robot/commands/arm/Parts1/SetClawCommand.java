// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.Parts1;

import frc.robot.commands.arm.ArmDefaultCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ClawState;

/** Add your docs here. */
public class SetClawCommand extends ArmDefaultCommand{

    public SetClawCommand(Arm arm, ClawState clawState) {
        super(arm);
        this.clawState = clawState;
    }

    @Override
    public void initialize() {
  
      this.elevatorPosition = arm.getElevatorPosition();
      this.elbowPosition = arm.getElbowPosition();
      this.elevatorTiltState = arm.getElevatorTiltState();
      this.ledAnimation = arm.getLEDAnimation();
      arm.setElevatorSetpoint(elevatorPosition);
      arm.setElbowSetPoint(elbowPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
