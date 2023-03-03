// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.ArmAutoFactories.ElementType;
import frc.robot.commands.ArmAutoFactories.PlacementType;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTilt;
import frc.robot.subsystems.Grabber;

/** Add your docs here. */
public class ArmCommandFactories {

        public static class Alignment {

                public static CommandBase createHigh(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt) {

                        CommandBase elevatorSetCommand = elevator
                                        .createControlCommand(Constants.Auto.Arm.Alignment.High.elevatorPositionMeters);
                        CommandBase elbowSetCommand = elbow
                                        .createControlCommand(Constants.Auto.Arm.Alignment.High.elbowPositionDegrees);
                        CommandBase elevatorAndArmAlignCommand = Commands.parallel(elevatorSetCommand, elbowSetCommand);

                        CommandBase tiltSetCommand = tilt
                                        .createSetStateCommand(Constants.Auto.Arm.Alignment.High.elevatorTiltState);
                        CommandBase waitForElevatorToTiltCommand = Commands
                                        .waitSeconds(Constants.Auto.Arm.Alignment.High.waitForElevatorToTilt);

                        return Commands.sequence(
                                        elevatorAndArmAlignCommand,
                                        tiltSetCommand,
                                        waitForElevatorToTiltCommand);
                }

                public static CommandBase createMiddle(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt) {

                        CommandBase elevatorSetCommand = elevator.createControlCommand(
                                        Constants.Auto.Arm.Alignment.Middle.elevatorPositionMeters);
                        CommandBase elbowSetCommand = elbow
                                        .createControlCommand(Constants.Auto.Arm.Alignment.Middle.elbowPositionDegrees);
                        CommandBase elevatorAndArmAlignCommand = Commands.parallel(elevatorSetCommand, elbowSetCommand);

                        CommandBase tiltSetCommand = tilt
                                        .createSetStateCommand(Constants.Auto.Arm.Alignment.Middle.elevatorTiltState);
                        CommandBase waitForElevatorToTiltCommand = Commands
                                        .waitSeconds(Constants.Auto.Arm.Alignment.Middle.waitForElevatorToTilt);

                        return Commands.sequence(
                                        elevatorAndArmAlignCommand,
                                        tiltSetCommand,
                                        waitForElevatorToTiltCommand);
                }

                public static CommandBase createLow(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt) {

                        CommandBase elevatorSetCommand = elevator.createControlCommand(
                                        Constants.Auto.Arm.Alignment.Low.elevatorPositionMeters);
                        CommandBase elbowSetCommand = elbow
                                        .createControlCommand(Constants.Auto.Arm.Alignment.Low.elbowPositionDegrees);
                        CommandBase elevatorAndArmAlignCommand = Commands.parallel(elevatorSetCommand, elbowSetCommand);

                        CommandBase tiltSetCommand = tilt
                                        .createSetStateCommand(Constants.Auto.Arm.Alignment.Low.elevatorTiltState);
                        CommandBase waitForElevatorToTiltCommand = Commands
                                        .waitSeconds(Constants.Auto.Arm.Alignment.Low.waitForElevatorToTilt);

                        return Commands.sequence(
                                        elevatorAndArmAlignCommand,
                                        tiltSetCommand,
                                        waitForElevatorToTiltCommand);
                }

        }

        public static class Placement {

                public static CommandBase create(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                Claw claw,
                                ElevatorTilt tilt,
                                ElementType elementType,
                                PlacementType placementType) {
                        if (elementType == ElementType.CONE) {
                                if (placementType == PlacementType.HIGH) {
                                        return Cone.createHigh(elevator, elbow, grabber, claw, tilt);
                                } else if (placementType == PlacementType.MIDDLE) {
                                        return Cone.createMiddle(elevator, elbow, grabber, claw, tilt);
                                } else {
                                        return Cone.createLow(elevator, elbow, grabber, claw, tilt);
                                }
                        } else {
                                if (placementType == PlacementType.HIGH) {
                                        return Cube.createHigh(elevator, elbow, grabber, claw, tilt);
                                } else if (placementType == PlacementType.MIDDLE) {
                                        return Cube.createMiddle(elevator, elbow, grabber, claw, tilt);
                                } else {
                                        return Cube.createLow(elevator, elbow, grabber, claw, tilt);
                                }
                        }

                }

                public static class Cone {
                        public static CommandBase createHigh(
                                        Elevator elevator,
                                        Elbow elbow,
                                        Grabber grabber,
                                        Claw claw,
                                        ElevatorTilt tilt) {
                                CommandBase tiltSetCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.High.Cone.elevatorTiltState);
                                CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(
                                                Constants.Auto.Arm.Placement.High.Cone.waitForElevatorToTilt);
                                CommandBase clawReleaseCommand = claw.createSetStateCommand(Claw.State.CUBE);

                                CommandBase elbowSetCommand = elbow.createControlCommand(
                                                Constants.Auto.Arm.Placement.elbowPositionAfterPlacement);
                                CommandBase tiltSetAfterCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.elevatorTiltStateAfterPlacement);
                                CommandBase elevatorSetCommand = elevator.createControlCommand(
                                                Constants.Auto.Arm.Placement.elevatorPositionAfterPlacement);

                                return Commands.sequence(
                                                tiltSetCommand,
                                                waitForElevatorToTiltCommand,
                                                clawReleaseCommand,
                                                elbowSetCommand,
                                                tiltSetAfterCommand,
                                                elevatorSetCommand);
                        }

                        public static CommandBase createMiddle(
                                        Elevator elevator,
                                        Elbow elbow,
                                        Grabber grabber,
                                        Claw claw,
                                        ElevatorTilt tilt) {
                                CommandBase tiltSetCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.Middle.Cone.elevatorTiltState);
                                CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(
                                                Constants.Auto.Arm.Placement.Middle.Cone.waitForElevatorToTilt);
                                CommandBase clawReleaseCommand = claw.createSetStateCommand(Claw.State.CUBE);

                                CommandBase elbowSetCommand = elbow.createControlCommand(
                                                Constants.Auto.Arm.Placement.elbowPositionAfterPlacement);
                                CommandBase tiltSetAfterCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.elevatorTiltStateAfterPlacement);
                                CommandBase elevatorSetCommand = elevator.createControlCommand(
                                                Constants.Auto.Arm.Placement.elevatorPositionAfterPlacement);

                                return Commands.sequence(
                                                tiltSetCommand,
                                                waitForElevatorToTiltCommand,
                                                clawReleaseCommand,
                                                elbowSetCommand,
                                                tiltSetAfterCommand,
                                                elevatorSetCommand);
                        }

                        public static CommandBase createLow(
                                        Elevator elevator,
                                        Elbow elbow,
                                        Grabber grabber,
                                        Claw claw,
                                        ElevatorTilt tilt) {
                                CommandBase tiltSetCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.Low.Cone.elevatorTiltState);
                                CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(
                                                Constants.Auto.Arm.Placement.Low.Cone.waitForElevatorToTilt);
                                CommandBase clawReleaseCommand = claw.createSetStateCommand(Claw.State.CUBE);

                                CommandBase elbowSetCommand = elbow.createControlCommand(
                                                Constants.Auto.Arm.Placement.elbowPositionAfterPlacement);
                                CommandBase tiltSetAfterCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.elevatorTiltStateAfterPlacement);
                                CommandBase elevatorSetCommand = elevator.createControlCommand(
                                                Constants.Auto.Arm.Placement.elevatorPositionAfterPlacement);

                                return Commands.sequence(
                                                tiltSetCommand,
                                                waitForElevatorToTiltCommand,
                                                clawReleaseCommand,
                                                elbowSetCommand,
                                                tiltSetAfterCommand,
                                                elevatorSetCommand);
                        }

                }

                public static class Cube {
                        public static CommandBase createHigh(
                                        Elevator elevator,
                                        Elbow elbow,
                                        Grabber grabber,
                                        Claw claw,
                                        ElevatorTilt tilt) {
                                CommandBase tiltSetCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.High.Cube.elevatorTiltState);
                                CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(
                                                Constants.Auto.Arm.Placement.High.Cube.waitForElevatorToTilt);

                                CommandBase clawReleaseCommand = grabber
                                                .createControlCommand(Constants.Auto.Arm.Placement.grabberSpeedCubeRPM);
                                CommandBase afterGrabberSet = Commands.waitSeconds(0.2);
                                CommandBase releaseCommand = Commands.race(clawReleaseCommand, afterGrabberSet);

                                CommandBase elbowSetCommand = elbow.createControlCommand(
                                                Constants.Auto.Arm.Placement.elbowPositionAfterPlacement);
                                CommandBase tiltSetAfterCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.elevatorTiltStateAfterPlacement);
                                CommandBase grabberOffCommand1 = grabber
                                                .createControlCommand(0);
                                CommandBase afterGrabberSet1 = Commands.waitSeconds(0.2);
                                CommandBase releaseCommand2 = Commands.race(grabberOffCommand1, afterGrabberSet1);

                                CommandBase elevatorSetCommand = elevator.createControlCommand(
                                                Constants.Auto.Arm.Placement.elevatorPositionAfterPlacement);

                                return Commands.sequence(
                                                tiltSetCommand,
                                                waitForElevatorToTiltCommand,
                                                releaseCommand,
                                                elbowSetCommand,
                                                tiltSetAfterCommand,
                                                releaseCommand2,
                                                elevatorSetCommand);
                        }

                        public static CommandBase createMiddle(
                                        Elevator elevator,
                                        Elbow elbow,
                                        Grabber grabber,
                                        Claw claw,
                                        ElevatorTilt tilt) {
                                CommandBase tiltSetCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.Middle.Cube.elevatorTiltState);
                                CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(
                                                Constants.Auto.Arm.Placement.Middle.Cube.waitForElevatorToTilt);
                                CommandBase clawReleaseCommand = grabber
                                                .createControlCommand(Constants.Auto.Arm.Placement.grabberSpeedCubeRPM);
                                CommandBase afterGrabberSet = Commands.waitSeconds(0.2);
                                CommandBase releaseCommand = Commands.race(clawReleaseCommand, afterGrabberSet);

                                CommandBase elbowSetCommand = elbow.createControlCommand(
                                                Constants.Auto.Arm.Placement.elbowPositionAfterPlacement);
                                CommandBase tiltSetAfterCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.elevatorTiltStateAfterPlacement);
                                CommandBase grabberOffCommand1 = grabber
                                                .createControlCommand(0);
                                CommandBase afterGrabberSet1 = Commands.waitSeconds(0.2);
                                CommandBase releaseCommand2 = Commands.race(grabberOffCommand1, afterGrabberSet1);
                                CommandBase elevatorSetCommand = elevator.createControlCommand(
                                                Constants.Auto.Arm.Placement.elevatorPositionAfterPlacement);

                                return Commands.sequence(
                                                tiltSetCommand,
                                                waitForElevatorToTiltCommand,
                                                releaseCommand,
                                                elbowSetCommand,
                                                tiltSetAfterCommand,
                                                releaseCommand2,
                                                elevatorSetCommand);
                        }

                        public static CommandBase createLow(
                                        Elevator elevator,
                                        Elbow elbow,
                                        Grabber grabber,
                                        Claw claw,
                                        ElevatorTilt tilt) {
                                CommandBase tiltSetCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.Low.Cube.elevatorTiltState);
                                CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(
                                                Constants.Auto.Arm.Placement.Low.Cube.waitForElevatorToTilt);
                                CommandBase clawReleaseCommand = grabber
                                                .createControlCommand(Constants.Auto.Arm.Placement.grabberSpeedCubeRPM);
                                CommandBase afterGrabberSet = Commands.waitSeconds(0.2);
                                CommandBase releaseCommand = Commands.race(clawReleaseCommand, afterGrabberSet);

                                CommandBase elbowSetCommand = elbow.createControlCommand(
                                                Constants.Auto.Arm.Placement.elbowPositionAfterPlacement);
                                CommandBase tiltSetAfterCommand = tilt.createSetStateCommand(
                                                Constants.Auto.Arm.Placement.elevatorTiltStateAfterPlacement);
                                CommandBase grabberOffCommand1 = grabber
                                                .createControlCommand(0);
                                CommandBase afterGrabberSet1 = Commands.waitSeconds(0.2);
                                CommandBase releaseCommand2 = Commands.race(grabberOffCommand1, afterGrabberSet1);
                                CommandBase elevatorSetCommand = elevator.createControlCommand(
                                                Constants.Auto.Arm.Placement.elevatorPositionAfterPlacement);

                                return Commands.sequence(
                                                tiltSetCommand,
                                                waitForElevatorToTiltCommand,
                                                releaseCommand,
                                                elbowSetCommand,
                                                tiltSetAfterCommand,
                                                releaseCommand2,
                                                elevatorSetCommand);
                        }

                }

                public static CommandBase createHigh(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                Claw claw,
                                ElevatorTilt tilt) {
                        CommandBase tiltSetCommand = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.High.Cone.elevatorTiltState);
                        CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(
                                        Constants.Auto.Arm.Placement.High.Cone.waitForElevatorToTilt);

                        BooleanSupplier clawOrPoof = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == Claw.State.CONE;
                                }

                        };

                        CommandBase clawReleaseCommand = claw.createSetStateCommand(Claw.State.CUBE);
                        CommandBase grabberPoofCommand = grabber
                                        .createControlCommand(Constants.Auto.Arm.Placement.grabberSpeedCubeRPM);

                        CommandBase waitForPoof = Commands.waitSeconds(0.2);
                        CommandBase poofCommand = Commands.race(grabberPoofCommand, waitForPoof);

                        CommandBase releaseCommand = Commands.either(clawReleaseCommand, poofCommand, clawOrPoof);

                        CommandBase elbowSetCommand = elbow.createControlCommand(
                                        Constants.Auto.Arm.Placement.elbowPositionAfterPlacement);
                        CommandBase tiltSetAfterCommand = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.elevatorTiltStateAfterPlacement);
                        CommandBase elevatorSetCommand = elevator.createControlCommand(
                                        Constants.Auto.Arm.Placement.elevatorPositionAfterPlacement);

                        return Commands.sequence(
                                        tiltSetCommand,
                                        waitForElevatorToTiltCommand,
                                        releaseCommand,
                                        elbowSetCommand,
                                        tiltSetAfterCommand,
                                        elevatorSetCommand);
                }

                public static CommandBase createMiddle(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                Claw claw,
                                ElevatorTilt tilt) {
                        CommandBase tiltSetCommand = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.Middle.Cone.elevatorTiltState);
                        CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(
                                        Constants.Auto.Arm.Placement.Middle.Cone.waitForElevatorToTilt);

                        BooleanSupplier clawOrPoof = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == Claw.State.CONE;
                                }

                        };

                        CommandBase clawReleaseCommand = claw.createSetStateCommand(Claw.State.CUBE);
                        CommandBase grabberPoofCommand = grabber
                                        .createControlCommand(Constants.Auto.Arm.Placement.grabberSpeedCubeRPM);

                        CommandBase waitForPoof = Commands.waitSeconds(0.2);
                        CommandBase poofCommand = Commands.race(grabberPoofCommand, waitForPoof);
                        CommandBase releaseCommand = Commands.either(clawReleaseCommand, poofCommand, clawOrPoof);

                        CommandBase elbowSetCommand = elbow.createControlCommand(
                                        Constants.Auto.Arm.Placement.elbowPositionAfterPlacement);
                        CommandBase tiltSetAfterCommand = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.elevatorTiltStateAfterPlacement);
                        CommandBase elevatorSetCommand = elevator.createControlCommand(
                                        Constants.Auto.Arm.Placement.elevatorPositionAfterPlacement);

                        return Commands.sequence(
                                        tiltSetCommand,
                                        waitForElevatorToTiltCommand,
                                        releaseCommand,
                                        elbowSetCommand,
                                        tiltSetAfterCommand,
                                        elevatorSetCommand);
                }

                public static CommandBase createLow(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                Claw claw,
                                ElevatorTilt tilt) {
                        CommandBase tiltSetCommand = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.Low.Cone.elevatorTiltState);
                        CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(
                                        Constants.Auto.Arm.Placement.Low.Cone.waitForElevatorToTilt);

                        BooleanSupplier clawOrPoof = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == Claw.State.CONE;
                                }

                        };

                        CommandBase clawReleaseCommand = claw.createSetStateCommand(Claw.State.CUBE);
                        CommandBase grabberPoofCommand = grabber
                                        .createControlCommand(Constants.Auto.Arm.Placement.grabberSpeedCubeRPM);

                        CommandBase waitForPoof = Commands.waitSeconds(0.2);
                        CommandBase poofCommand = Commands.race(grabberPoofCommand, waitForPoof);
                        CommandBase releaseCommand = Commands.either(clawReleaseCommand, poofCommand, clawOrPoof);

                        CommandBase elbowSetCommand = elbow.createControlCommand(
                                        Constants.Auto.Arm.Placement.elbowPositionAfterPlacement);
                        CommandBase tiltSetAfterCommand = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.elevatorTiltStateAfterPlacement);
                        CommandBase elevatorSetCommand = elevator.createControlCommand(
                                        Constants.Auto.Arm.Placement.elevatorPositionAfterPlacement);

                        return Commands.sequence(
                                        tiltSetCommand,
                                        waitForElevatorToTiltCommand,
                                        releaseCommand,
                                        elbowSetCommand,
                                        tiltSetAfterCommand,
                                        elevatorSetCommand);
                }

        }

        public static class Pickup {

                public static CommandBase create(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                Claw claw,
                                ElevatorTilt tilt,
                                double elevatorPositionMeters,
                                double elbowPositionDegrees,
                                double grabberSpeedRPM,
                                ElevatorTilt.State elevatorTiltState,
                                Claw.State clawState) {

                        CommandBase elevatorSetCommand = elevator.createControlCommand(elevatorPositionMeters);
                        CommandBase elbowSetCommand = elbow.createControlCommand(elbowPositionDegrees);
                        CommandBase grabberSetCommand = grabber.createControlCommand(grabberSpeedRPM);
                        CommandBase tiltSetCommand = tilt.createSetStateCommand(elevatorTiltState);
                        CommandBase clawSetCommand = claw.createSetStateCommand(clawState);

                        return Commands.parallel(
                                        elevatorSetCommand,
                                        elbowSetCommand,
                                        grabberSetCommand,
                                        tiltSetCommand,
                                        clawSetCommand);

                }
        }

        public static class Carry {

                public static CommandBase create(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                ElevatorTilt tilt,
                                double elevatorPositionMeters,
                                double elbowPositionDegrees,
                                ElevatorTilt.State elevatorTiltState) {

                        CommandBase elevatorSetCommand = elevator.createControlCommand(elevatorPositionMeters);
                        CommandBase elbowSetCommand = elbow.createControlCommand(elbowPositionDegrees);
                        CommandBase grabberSetCommand = grabber.createStopCommand();
                        CommandBase tiltSetCommand = tilt.createSetStateCommand(elevatorTiltState);

                        return Commands.parallel(
                                        elevatorSetCommand,
                                        elbowSetCommand,
                                        grabberSetCommand,
                                        tiltSetCommand);
                }
        }

}
