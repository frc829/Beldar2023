// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.ManualControl;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.controls.ManualSpeedControl;
import frc.robot.subsystems.Arm;

public class ManualControlCommand extends CommandBase {

  protected final Arm arm;
  protected final ManualSpeedControl elevatorManualSpeedControl;
  protected final ManualSpeedControl elbowManualSpeedControl;
  protected final ManualSpeedControl grabberManualSpeedControl;

  public ManualControlCommand(
      Arm arm) {
    this.arm = arm;
    this.elevatorManualSpeedControl = arm.getElevatorManualSpeedControl();
    this.elbowManualSpeedControl = arm.getElbowManualSpeedControl();
    this.grabberManualSpeedControl = arm.getGrabbManualSpeedControl();
    addRequirements(arm);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elevatorSpeed = elevatorManualSpeedControl.getManualSpeed();
    double elbowSpeed = elbowManualSpeedControl.getManualSpeed();
    double grabberSpeed = grabberManualSpeedControl.getManualSpeed();
    if (elevatorSpeed == 0) {
      arm.setElevatorSetpoint(arm.getElevatorPosition());
      elevatorSpeed = arm.calculateElevatorSpeed(arm.getElevatorPosition());
    }

    if (elbowSpeed == 0) {
      arm.setElbowSetPoint(arm.getElbowPosition());
      elbowSpeed = arm.calculateElbowSpeed(arm.getElbowPosition());
    }

    arm.setElevatorVelocity(elevatorSpeed);
    arm.setElbowVelocity(elbowSpeed);
    arm.setAverageVelocityPerSecond(grabberSpeed);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setAverageVelocityPerSecond(0);
    arm.setElevatorVelocity(0);
    arm.setElbowVelocity(0);
  }

}
