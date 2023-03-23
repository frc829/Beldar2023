// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandSequences.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.Parts3.SetElevatorElbowTiltCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ElevatorTiltState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignMiddleForCone extends SequentialCommandGroup {

  private final double elevatorPositionMeters = 0.495;
  private final double elbowPositionDegrees = 29.0;
  private final ElevatorTiltState elevatorTiltState = ElevatorTiltState.SIX;

  public AlignMiddleForCone(Arm arm) {

    SetElevatorElbowTiltCommand setElevatorElbowTiltCommand = new SetElevatorElbowTiltCommand(
      arm, 
      elevatorPositionMeters, 
      elbowPositionDegrees, 
      elevatorTiltState);

    addCommands(setElevatorElbowTiltCommand);
  }
}
