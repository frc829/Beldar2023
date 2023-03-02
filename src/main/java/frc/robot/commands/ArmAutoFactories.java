// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
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

            double elevatorPositionMeters = getElevatorPositionMeters(placementType);
            double elbowPositionDegrees = getElbowPositionDegrees(placementType);
            ElevatorTilt.State elevatorTiltState = getElevatorTiltState(placementType);
            double waitForElevatorToTilt = getWaitForElevatorToTilt(placementType);
            double grabberSpeedRPMCube = getGrabberSpeedCubeRPM();
            double grabberWaitTimeCube = getGrabberWaitTimeCube();
            double waitForElevatorToTilt2 = getWaitForElevatorToTilt2(placementType);
            ElevatorTilt.State elevatorTiltState2 = getElevatorTiltState2(placementType);
            ElevatorTilt.State elevatorTiltState3 = getElevatorTiltState3();
            CommandBase alignment = Alignment.create(
                    elevator,
                    elbow,
                    tilt,
                    elevatorPositionMeters,
                    elbowPositionDegrees,
                    elevatorTiltState,
                    waitForElevatorToTilt);

            CommandBase waitForAlignment = Commands.waitSeconds(.5);

            CommandBase placement = Placement.create(
                elevator, 
                elbow, 
                grabber, 
                claw, 
                tilt, 
                grabberSpeedRPMCube, 
                grabberWaitTimeCube, 
                waitForElevatorToTilt2, 
                elevatorPositionMeters, 
                Constants.Auto.Arm.Placement.elbowPositionAfterPlacement, 
                elevatorTiltState2, 
                elevatorTiltState3);
                
            return Commands.sequence(alignment, waitForAlignment, placement);

        }

        private static ElevatorTilt.State getElevatorTiltState2(
                PlacementType placementType) {
            if (placementType == PlacementType.HIGH) {
                return Constants.Auto.Arm.Placement.High.elevatorTiltState;
            } else if (placementType == PlacementType.MIDDLE) {
                return Constants.Auto.Arm.Placement.Middle.elevatorTiltState;
            } else {
                return Constants.Auto.Arm.Placement.Low.elevatorTiltState;
            }
        }

        private static ElevatorTilt.State getElevatorTiltState3() {

            return Constants.Auto.Arm.Placement.elevatorTiltStateAfterPlacement;
        }

        private static double getWaitForElevatorToTilt2(PlacementType placementType) {
            if (placementType == PlacementType.HIGH) {
                return Constants.Auto.Arm.Placement.High.waitForElevatorToTilt;
            } else if (placementType == PlacementType.MIDDLE) {
                return Constants.Auto.Arm.Placement.Middle.waitForElevatorToTilt;
            } else {
                return Constants.Auto.Arm.Placement.Low.waitForElevatorToTilt;
            }
        }

        private static double getGrabberWaitTimeCube() {
            return Constants.Auto.Arm.Placement.grabberSpeedWaitTimeCube;
        }

        private static double getGrabberSpeedCubeRPM() {
            return Constants.Auto.Arm.Placement.grabberSpeedCubeRPM;
        }

        private static double getWaitForElevatorToTilt(PlacementType placementType) {
            if (placementType == PlacementType.HIGH) {
                return Constants.Auto.Arm.Alignment.High.waitForElevatorToTilt;
            } else if (placementType == PlacementType.MIDDLE) {
                return Constants.Auto.Arm.Alignment.Middle.waitForElevatorToTilt;
            } else {
                return Constants.Auto.Arm.Alignment.Low.waitForElevatorToTilt;
            }
        }

        private static ElevatorTilt.State getElevatorTiltState(PlacementType placementType) {
            if (placementType == PlacementType.HIGH) {
                return Constants.Auto.Arm.Alignment.High.elevatorTiltState;
            } else if (placementType == PlacementType.MIDDLE) {
                return Constants.Auto.Arm.Alignment.Middle.elevatorTiltState;
            } else {
                return Constants.Auto.Arm.Alignment.Low.elevatorTiltState;
            }
        }

        private static double getElevatorPositionMeters(PlacementType placementType) {
            if (placementType == PlacementType.HIGH) {
                return Constants.Auto.Arm.Alignment.High.elevatorPositionMeters;
            } else if (placementType == PlacementType.MIDDLE) {
                return Constants.Auto.Arm.Alignment.Middle.elevatorPositionMeters;
            } else {
                return Constants.Auto.Arm.Alignment.Low.elevatorPositionMeters;
            }
        }

        private static double getElbowPositionDegrees(PlacementType placementType) {
            if (placementType == PlacementType.HIGH) {
                return Constants.Auto.Arm.Alignment.High.elbowPositionDegrees;
            } else if (placementType == PlacementType.MIDDLE) {
                return Constants.Auto.Arm.Alignment.Middle.elbowPositionDegrees;
            } else {
                return Constants.Auto.Arm.Alignment.Low.elbowPositionDegrees;
            }
        }
    }

}
