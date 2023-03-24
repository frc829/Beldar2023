// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandSequences.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.Parts6.SetElevatorElbowTiltClawGrabberLEDLightsCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ClawState;
import frc.robot.subsystems.Arm.ElevatorTiltState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupFloorCone extends SequentialCommandGroup {

  private final double elevatorPositionMeters = 0.0;
  private final double elbowPositionDegrees = 106.5;
  private final ElevatorTiltState elevatorTiltState = ElevatorTiltState.TWO;
  private final double grabberSpeedRPM = 916.0;
  private final ClawState clawState = ClawState.CONE;

  public PickupFloorCone(Arm arm) {

    SetElevatorElbowTiltClawGrabberLEDLightsCommand setElevatorElbowTiltClawGrabberLEDLightsCommand = new SetElevatorElbowTiltClawGrabberLEDLightsCommand(
    arm, 
    elevatorPositionMeters, 
    elbowPositionDegrees, 
    elevatorTiltState, 
    clawState, 
    grabberSpeedRPM, 
    null);

    addCommands(setElevatorElbowTiltClawGrabberLEDLightsCommand);
  }
}
