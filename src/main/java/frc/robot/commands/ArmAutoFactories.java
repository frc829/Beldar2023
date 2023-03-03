// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArmCommandFactories.Alignment;
import frc.robot.commands.ArmCommandFactories.Placement;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTilt;
import frc.robot.subsystems.Grabber;

/** Add your docs here. */
public class ArmAutoFactories {
    public enum ElementType {
        CONE, CUBE
    }

    public enum PlacementType {
        HIGH, MIDDLE, LOW
    }

    public enum PickupType {
        Floor, Sliding, Drop
    }

    public static class AlignmentAndPlacement {
        public static CommandBase create(
                Elevator elevator,
                Elbow elbow,
                Grabber grabber,
                Claw claw,
                ElevatorTilt tilt,
                ElementType elementType,
                PlacementType placementType) {

            if (placementType == PlacementType.HIGH) {
                CommandBase alignment = Alignment.createHigh(elevator, elbow, tilt);
                CommandBase waitForAlignment = Commands.waitSeconds(.5);
                CommandBase placement = Placement.create(elevator, elbow, grabber, claw, tilt, elementType,placementType);
                return Commands.sequence(alignment, waitForAlignment, placement);
            } else if (placementType == PlacementType.MIDDLE) {
                CommandBase alignment = Alignment.createMiddle(elevator, elbow, tilt);
                CommandBase waitForAlignment = Commands.waitSeconds(.5);
                CommandBase placement = Placement.create(elevator, elbow, grabber, claw, tilt, elementType,placementType);
                return Commands.sequence(alignment, waitForAlignment, placement);
            }
            else{
                CommandBase alignment = Alignment.createLow(elevator, elbow, tilt);
                CommandBase waitForAlignment = Commands.waitSeconds(.5);
                CommandBase placement = Placement.create(elevator, elbow, grabber, claw, tilt, elementType,placementType);
                return Commands.sequence(alignment, waitForAlignment, placement);
            }

        }
    }

}
