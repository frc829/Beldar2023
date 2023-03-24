// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandSequences.arm;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.Parts1.SetGrabberCommand;
import frc.robot.commands.arm.Parts1.SetTiltCommand;
import frc.robot.commands.arm.Parts4.SetElevatorElbowTiltGrabberCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ElevatorTiltState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMiddleCube extends SequentialCommandGroup {

  private final ElevatorTiltState elevatorTiltState0 = ElevatorTiltState.TWO;
  private final ElevatorTiltState elevatorTiltState1 = ElevatorTiltState.NONE;
  private final double grabberSpeedRPM = -916.0;
  private final double waitForRelease = 0.25;
  private final double elevatorPosition = 0.0;
  private final double elbowPositionDegrees = 30.0;
  

  public ScoreMiddleCube(Arm arm) {

    SetTiltCommand setTiltCommand = new SetTiltCommand(
        arm,
        elevatorTiltState0);

    SetGrabberCommand setGrabberCommand = new SetGrabberCommand(
      arm, 
      grabberSpeedRPM);

    WaitCommand waitForReleaseCommand = new WaitCommand(waitForRelease);

    ParallelRaceGroup grabberRelease = new ParallelRaceGroup(
      setGrabberCommand, 
      waitForReleaseCommand);

    SetElevatorElbowTiltGrabberCommand setElevatorElbowTiltGrabberCommand = new SetElevatorElbowTiltGrabberCommand(
      arm, 
      elevatorPosition, 
      elbowPositionDegrees / 360.0, 
      elevatorTiltState1, 
      0.0);

    addCommands(setTiltCommand, grabberRelease, setElevatorElbowTiltGrabberCommand);
  }
}
