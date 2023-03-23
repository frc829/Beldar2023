// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.lighting.animations.LEDAnimation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ClawState;
import frc.robot.subsystems.Arm.ElevatorTiltState;

public class ArmDefaultCommand extends CommandBase {
  protected final Arm arm;

  protected double elevatorPosition = 0;
  protected double elbowPosition = 0;
  protected ClawState clawState = ClawState.CUBE;
  protected ElevatorTiltState elevatorTiltState = ElevatorTiltState.NONE;
  protected double grabberSpeed = 0;
  protected LEDAnimation ledAnimation = null;

  public ArmDefaultCommand(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.elevatorPosition = arm.getElevatorPosition();
    this.elbowPosition = arm.getElbowPosition();
    this.clawState = arm.getClawState();
    this.elevatorTiltState = arm.getElevatorTiltState();
    this.ledAnimation = arm.getLEDAnimation();

    arm.setElevatorSetpoint(elevatorPosition);
    arm.setElbowSetPoint(elbowPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(clawState != arm.getClawState()){
      arm.setClawState(clawState);
    }
    if(elevatorTiltState != arm.getElevatorTiltState()){
      arm.setElevatorTiltState(elevatorTiltState);
    }
    if(ledAnimation != arm.getLEDAnimation()){
      arm.setLEDAnimation(ledAnimation);
    }

    arm.setAverageVelocityPerSecond(grabberSpeed);
    double currentElevatorPosition = arm.getElevatorPosition();
    double elevatorSpeedMetersPerSecond = arm.calculateElevatorSpeed(currentElevatorPosition);
    double currentElbowPosition = arm.getElbowPosition();
    double elbowSpeedRotationsPerSecond = arm.calculateElbowSpeed(currentElbowPosition);
    arm.setElevatorVelocity(elevatorSpeedMetersPerSecond);
    arm.setElbowVelocity(elbowSpeedRotationsPerSecond);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
