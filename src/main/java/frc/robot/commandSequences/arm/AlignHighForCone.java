// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandSequences.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.Parts1.SetTiltCommand;
import frc.robot.commands.arm.Parts2.SetElevatorElbowCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ElevatorTiltState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignHighForCone extends SequentialCommandGroup {

  private final double elevatorPositionMeters0 = 0.75;
  private final double elbowPositionDegrees0 = 50.0;
  private final double elevatorPositionMeters1 = 0.914;
  private final double elbowPositionDegrees1 = 6.5;
  private final ElevatorTiltState elevatorTiltState = ElevatorTiltState.SIX;
  private final double waitForElevatorToTilt = 0.5;

  public AlignHighForCone(Arm arm) {

    SetElevatorElbowCommand setElevatorElbowCommand0 = new SetElevatorElbowCommand(
        arm,
        elevatorPositionMeters0,
        elbowPositionDegrees0 / 360.0);

    SetElevatorElbowCommand setElevatorElbowCommand1 = new SetElevatorElbowCommand(
        arm,
        elevatorPositionMeters1,
        elbowPositionDegrees1 / 360.0);

    SetTiltCommand setTiltCommand = new SetTiltCommand(
        arm,
        elevatorTiltState);

    WaitCommand waitAfterTilt = new WaitCommand(waitForElevatorToTilt);

    addCommands(setElevatorElbowCommand0, setElevatorElbowCommand1, setTiltCommand, waitAfterTilt);
  }
}
