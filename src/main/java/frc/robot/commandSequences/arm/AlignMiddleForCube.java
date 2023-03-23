// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandSequences.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.Parts2.SetElevatorElbowCommand;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignMiddleForCube extends SequentialCommandGroup {

  private final double elevatorPositionMeters = 0;
  private final double elbowPositionDegrees = 40.0;

  public AlignMiddleForCube(Arm arm) {

    SetElevatorElbowCommand setElevatorElbowCommand = new SetElevatorElbowCommand(
      arm, 
      elevatorPositionMeters, 
      elbowPositionDegrees);

    addCommands(setElevatorElbowCommand);
  }
}
