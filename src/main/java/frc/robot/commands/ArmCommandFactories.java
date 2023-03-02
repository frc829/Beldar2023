// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTilt;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Claw.State;

/** Add your docs here. */
public class ArmCommandFactories {

        public static class Alignment {
                public static CommandBase create(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                double elevatorPositionMeters,
                                double elbowPositionDegrees,
                                ElevatorTilt.State elevatorTiltState,
                                double waitForElevatorToTilt) {

                        CommandBase elevatorSetCommand = elevator.createControlCommand(elevatorPositionMeters);
                        CommandBase elbowSetCommand = elbow.createControlCommand(elbowPositionDegrees);
                        CommandBase elevatorAndArmAlignCommand = Commands.parallel(elevatorSetCommand, elbowSetCommand);

                        CommandBase tiltSetCommand = tilt.createSetStateCommand(elevatorTiltState);
                        CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(waitForElevatorToTilt);

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
                                double grabberSpeedRPMCube,
                                double grabberWaitTimeCube,
                                double waitForElevatorToTilt,
                                double elevatorPositionAfterPlacement,
                                double elbowPositionAfterPlacement,
                                ElevatorTilt.State elevatorTiltState,
                                ElevatorTilt.State elevatorTiltStateAfterPlacement) {

                        CommandBase tiltSetCommand = tilt.createSetStateCommand(elevatorTiltState);
                        CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(waitForElevatorToTilt);

                        BooleanSupplier isCone = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == State.CONE ? true : false;
                                }

                        };

                        CommandBase clawReleaseConeCommand = claw.createSetStateCommand(Claw.State.CUBE);

                        CommandBase grabberSetCubeCommand = grabber.createControlCommand(grabberSpeedRPMCube);
                        CommandBase grabberWaitTimeCubeCommand = Commands.waitSeconds(grabberWaitTimeCube);
                        CommandBase grabberReleaseCubeCommand = Commands.race(grabberSetCubeCommand,
                                        grabberWaitTimeCubeCommand);

                        CommandBase releaseCommand = Commands.either(clawReleaseConeCommand, grabberReleaseCubeCommand,
                                        isCone);

                        CommandBase elbowSetCommand = elbow.createControlCommand(elbowPositionAfterPlacement);
                        CommandBase grabberSetAfterCommand = grabber.createStopCommand();
                        CommandBase tiltSetAfterCommand = tilt.createSetStateCommand(elevatorTiltStateAfterPlacement);
                        CommandBase elevatorSetCommand = elevator.createControlCommand(elevatorPositionAfterPlacement);

                        return Commands.sequence(
                                        tiltSetCommand,
                                        waitForElevatorToTiltCommand,
                                        releaseCommand,
                                        elbowSetCommand,
                                        grabberSetAfterCommand,
                                        tiltSetAfterCommand,
                                        elevatorSetCommand);
                }

                public static CommandBase create(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                Claw claw,
                                ElevatorTilt tilt,
                                double grabberSpeedRPMCube,
                                double grabberWaitTimeCube,
                                double waitForElevatorToTilt,
                                ElevatorTilt.State elevatorTiltState,
                                ElevatorTilt.State elevatorTiltStateAfterPlacement) {

                        CommandBase tiltSetCommand = tilt.createSetStateCommand(elevatorTiltState);
                        CommandBase waitForElevatorToTiltCommand = Commands.waitSeconds(waitForElevatorToTilt);

                        BooleanSupplier isCone = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == State.CONE ? true : false;
                                }

                        };

                        CommandBase clawReleaseConeCommand = claw.createSetStateCommand(Claw.State.CUBE);

                        CommandBase grabberSetCubeCommand = grabber.createControlCommand(grabberSpeedRPMCube);
                        CommandBase grabberWaitTimeCubeCommand = Commands.waitSeconds(grabberWaitTimeCube);
                        CommandBase grabberReleaseCubeCommand = Commands.race(grabberSetCubeCommand,
                                        grabberWaitTimeCubeCommand);

                        CommandBase releaseCommand = Commands.either(clawReleaseConeCommand, grabberReleaseCubeCommand,
                                        isCone);

                        CommandBase grabberSetAfterCommand = grabber.createStopCommand();
                        CommandBase tiltSetAfterCommand = tilt.createSetStateCommand(elevatorTiltStateAfterPlacement);

                        return Commands.sequence(
                                        tiltSetCommand,
                                        waitForElevatorToTiltCommand,
                                        releaseCommand,
                                        grabberSetAfterCommand,
                                        tiltSetAfterCommand);
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
