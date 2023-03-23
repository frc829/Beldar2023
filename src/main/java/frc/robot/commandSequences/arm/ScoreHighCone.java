// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandSequences.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.Parts1.SetClawCommand;
import frc.robot.commands.arm.Parts1.SetTiltCommand;
import frc.robot.commands.arm.Parts3.SetElevatorElbowTiltCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ClawState;
import frc.robot.subsystems.Arm.ElevatorTiltState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighCone extends SequentialCommandGroup {

  private final ElevatorTiltState elevatorTiltState0 = ElevatorTiltState.EIGHT;
  private final double waitForElevatorToTilt = 0.5;
  private final ElevatorTiltState elevatorTiltState1 = ElevatorTiltState.NONE;
  private final double elevatorPosition = 0.0;
  private final double elbowPositionDegrees = 30.0;
  

  public ScoreHighCone(Arm arm) {

    SetTiltCommand setTiltCommand = new SetTiltCommand(
        arm,
        elevatorTiltState0);

    WaitCommand waitAfterTilt = new WaitCommand(waitForElevatorToTilt);

    SetClawCommand setClawCommand = new SetClawCommand(
        arm,
        ClawState.CUBE);

    SetElevatorElbowTiltCommand setElevatorElbowTiltCommand = new SetElevatorElbowTiltCommand(
        arm,
        elevatorPosition,
        elbowPositionDegrees / 360.0,
        elevatorTiltState1);

    addCommands(setTiltCommand, waitAfterTilt, setClawCommand, setElevatorElbowTiltCommand);
  }
}
