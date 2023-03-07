// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTilt;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.ElevatorTilt.State;

/** Add your docs here. */
public class Arm {

        public static class Alignment {

                public static CommandBase createHigh(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw) {

                        CommandBase elevatorSetCommand0 = elevator.createConeCubeCommand(
                                        Constants.Auto.Arm.Alignment.High.Cone.elevatorPositionMeters,
                                        Constants.Auto.Arm.Alignment.High.Cube.elevatorPositionMeters,
                                        claw::hasCone);

                        CommandBase elbowSetCommand0 = elbow.createConeCubeCommand(
                                        Constants.Auto.Arm.Alignment.High.Cone.elbowPositionDegrees,
                                        Constants.Auto.Arm.Alignment.High.Cube.elbowPositionDegrees,
                                        claw::hasCone);

                        CommandBase tiltElevatorCommand0 = tilt.createConeCubeCommand(
                                        Constants.Auto.Arm.Alignment.High.Cone.elevatorTiltState,
                                        Constants.Auto.Arm.Alignment.High.Cube.elevatorTiltState,
                                        claw::hasCone);

                        BooleanSupplier elbowAndElevatorStopCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return elevator.atSetpoint() && elbow.atSetpoint();
                                }

                        };

                        CommandBase elbowAndElevatorDeadline = Commands.waitUntil(elbowAndElevatorStopCondition);

                        return Commands.deadline(
                                        elbowAndElevatorDeadline,
                                        elevatorSetCommand0,
                                        elbowSetCommand0,
                                        tiltElevatorCommand0);
                }

                public static CommandBase createMiddle(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw) {

                        CommandBase elevatorSetCommand0 = elevator.createConeCubeCommand(
                                        Constants.Auto.Arm.Alignment.Middle.Cone.elevatorPositionMeters,
                                        Constants.Auto.Arm.Alignment.Middle.Cube.elevatorPositionMeters,
                                        claw::hasCone);

                        CommandBase elbowSetCommand0 = elbow.createConeCubeCommand(
                                        Constants.Auto.Arm.Alignment.Middle.Cone.elbowPositionDegrees,
                                        Constants.Auto.Arm.Alignment.Middle.Cube.elbowPositionDegrees,
                                        claw::hasCone);

                        CommandBase tiltElevatorCommand0 = tilt.createConeCubeCommand(
                                        Constants.Auto.Arm.Alignment.Middle.Cone.elevatorTiltState,
                                        Constants.Auto.Arm.Alignment.Middle.Cube.elevatorTiltState,
                                        claw::hasCone);

                        BooleanSupplier elbowAndElevatorStopCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return elevator.atSetpoint() && elbow.atSetpoint();
                                }

                        };

                        CommandBase elbowAndElevatorDeadline = Commands.waitUntil(elbowAndElevatorStopCondition);

                        return Commands.deadline(
                                        elbowAndElevatorDeadline,
                                        elevatorSetCommand0,
                                        elbowSetCommand0,
                                        tiltElevatorCommand0);
                }

                public static CommandBase createLow(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw) {

                        CommandBase elevatorSetCommand0 = elevator.createConeCubeCommand(
                                        Constants.Auto.Arm.Alignment.Low.Cone.elevatorPositionMeters,
                                        Constants.Auto.Arm.Alignment.Low.Cube.elevatorPositionMeters,
                                        claw::hasCone);

                        CommandBase elbowSetCommand0 = elbow.createConeCubeCommand(
                                        Constants.Auto.Arm.Alignment.Low.Cone.elbowPositionDegrees,
                                        Constants.Auto.Arm.Alignment.Low.Cube.elbowPositionDegrees,
                                        claw::hasCone);

                        CommandBase tiltElevatorCommand0 = tilt.createConeCubeCommand(
                                        Constants.Auto.Arm.Alignment.Low.Cone.elevatorTiltState,
                                        Constants.Auto.Arm.Alignment.Low.Cube.elevatorTiltState,
                                        claw::hasCone);

                        BooleanSupplier elbowAndElevatorStopCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return elevator.atSetpoint() && elbow.atSetpoint();
                                }

                        };

                        CommandBase elbowAndElevatorDeadline = Commands.waitUntil(elbowAndElevatorStopCondition);

                        return Commands.deadline(
                                        elbowAndElevatorDeadline,
                                        elevatorSetCommand0,
                                        elbowSetCommand0,
                                        tiltElevatorCommand0);
                }

        }

        public static class Placement {

                public static CommandBase createHigh(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw,
                                Grabber grabber) {

                        CommandBase tiltElevatorCommand0 = tilt.createConeCubeCommand(
                                        Constants.Auto.Arm.Placement.High.Cone.elevatorTiltState,
                                        Constants.Auto.Arm.Placement.High.Cube.elevatorTiltState,
                                        claw::hasCone);

                        CommandBase waitForTiltCone = Commands
                                        .waitSeconds(Constants.Auto.Arm.Placement.High.Cone.waitForElevatorToTilt);
                        CommandBase waitForTiltCube = Commands
                                        .waitSeconds(Constants.Auto.Arm.Placement.High.Cube.waitForElevatorToTilt);
                        CommandBase waitForTilt0 = Commands.either(waitForTiltCone, waitForTiltCube, claw::hasCone);

                        CommandBase releaseCone = claw.createSetStateCommand(Claw.State.CUBE);
                        CommandBase releaseCube = grabber.createControlCommand(
                                        Constants.Auto.Arm.Placement.High.Cube.grabberSpeedCubeRPM);
                        CommandBase release0 = Commands.either(releaseCone, releaseCube, claw::hasCone);

                        return Commands.sequence(
                                        tiltElevatorCommand0,
                                        waitForTilt0,
                                        release0);
                }

                public static CommandBase createMiddle(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw,
                                Grabber grabber) {

                        CommandBase tiltElevatorCommand0 = tilt.createConeCubeCommand(
                                        Constants.Auto.Arm.Placement.Middle.Cone.elevatorTiltState,
                                        Constants.Auto.Arm.Placement.Middle.Cube.elevatorTiltState,
                                        claw::hasCone);

                        CommandBase waitForTiltCone = Commands
                                        .waitSeconds(Constants.Auto.Arm.Placement.Middle.Cone.waitForElevatorToTilt);
                        CommandBase waitForTiltCube = Commands
                                        .waitSeconds(Constants.Auto.Arm.Placement.Middle.Cube.waitForElevatorToTilt);
                        CommandBase waitForTilt0 = Commands.either(waitForTiltCone, waitForTiltCube, claw::hasCone);

                        CommandBase releaseCone = claw.createSetStateCommand(Claw.State.CUBE);
                        CommandBase releaseCube = grabber.createControlCommand(
                                        Constants.Auto.Arm.Placement.Middle.Cube.grabberSpeedCubeRPM);
                        CommandBase release0 = Commands.either(releaseCone, releaseCube, claw::hasCone);

                        return Commands.sequence(
                                        tiltElevatorCommand0,
                                        waitForTilt0,
                                        release0);

                }

                public static CommandBase createLow(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw,
                                Grabber grabber) {

                        CommandBase tiltElevatorCommand0 = tilt.createConeCubeCommand(
                                        Constants.Auto.Arm.Placement.Low.Cone.elevatorTiltState,
                                        Constants.Auto.Arm.Placement.Low.Cube.elevatorTiltState,
                                        claw::hasCone);

                        CommandBase waitForTiltCone = Commands
                                        .waitSeconds(Constants.Auto.Arm.Placement.Low.Cone.waitForElevatorToTilt);
                        CommandBase waitForTiltCube = Commands
                                        .waitSeconds(Constants.Auto.Arm.Placement.Low.Cube.waitForElevatorToTilt);
                        CommandBase waitForTilt0 = Commands.either(waitForTiltCone, waitForTiltCube, claw::hasCone);

                        CommandBase releaseCone = claw.createSetStateCommand(Claw.State.CUBE);
                        CommandBase releaseCube = grabber.createControlCommand(
                                        Constants.Auto.Arm.Placement.Low.Cube.grabberSpeedCubeRPM);
                        CommandBase release0 = Commands.either(releaseCone, releaseCube, claw::hasCone);

                        return Commands.sequence(
                                        tiltElevatorCommand0,
                                        waitForTilt0,
                                        release0);

                }
        }

        public static class Reset {
                public static CommandBase createHigh(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Grabber grabber) {

                        CommandBase elevatorSetCommand0 = elevator.createControlCommand(
                                        Constants.Auto.Arm.Reset.High.Cone.elevatorPosition1);

                        CommandBase elbowSetCommand0 = elbow.createControlCommand(
                                        Constants.Auto.Arm.Reset.High.Cone.elbowPosition);

                        CommandBase tiltSetCommand0 = tilt.createSetStateCommand(State.NONE);

                        CommandBase grabberStopCommand = grabber.createStopCommand();

                        BooleanSupplier elbowAndElevatorStopCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return elevator.atSetpoint() && elbow.atSetpoint();
                                }

                        };

                        CommandBase elbowAndElevatorDeadline = Commands.waitUntil(elbowAndElevatorStopCondition);

                        return Commands.deadline(
                                        elbowAndElevatorDeadline,
                                        elevatorSetCommand0,
                                        elbowSetCommand0,
                                        tiltSetCommand0,
                                        grabberStopCommand);
                }

                public static CommandBase createMiddle(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Grabber grabber) {

                        CommandBase elevatorSetCommand0 = elevator.createControlCommand(
                                        Constants.Auto.Arm.Reset.Middle.Cone.elevatorPosition);

                        CommandBase elbowSetCommand0 = elbow.createControlCommand(
                                        Constants.Auto.Arm.Reset.Middle.Cone.elbowPosition);

                        CommandBase tiltSetCommand0 = tilt.createSetStateCommand(State.NONE);

                        CommandBase grabberStopCommand = grabber.createStopCommand();

                        BooleanSupplier elbowAndElevatorStopCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return elevator.atSetpoint() && elbow.atSetpoint();
                                }

                        };

                        CommandBase elbowAndElevatorDeadline = Commands.waitUntil(elbowAndElevatorStopCondition);

                        return Commands.deadline(
                                        elbowAndElevatorDeadline,
                                        elevatorSetCommand0,
                                        elbowSetCommand0,
                                        tiltSetCommand0,
                                        grabberStopCommand);

                }

                public static CommandBase createLow(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Grabber grabber) {

                        CommandBase elevatorSetCommand0 = elevator.createControlCommand(
                                        Constants.Auto.Arm.Reset.Low.Cone.elevatorPosition);

                        CommandBase elbowSetCommand0 = elbow.createControlCommand(
                                        Constants.Auto.Arm.Reset.Low.Cone.elbowPosition);

                        CommandBase tiltSetCommand0 = tilt.createSetStateCommand(State.NONE);

                        CommandBase grabberStopCommand = grabber.createStopCommand();

                        BooleanSupplier elbowAndElevatorStopCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return elevator.atSetpoint() && elbow.atSetpoint();
                                }

                        };

                        CommandBase elbowAndElevatorDeadline = Commands.waitUntil(elbowAndElevatorStopCondition);

                        return Commands.deadline(
                                        elbowAndElevatorDeadline,
                                        elevatorSetCommand0,
                                        elbowSetCommand0,
                                        tiltSetCommand0,
                                        grabberStopCommand);
                }
        }

        public static class Pickup {

                public static CommandBase createDrop(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                Claw claw,
                                ElevatorTilt tilt,
                                Claw.State clawState) {

                        CommandBase clawSetCommand0 = claw.createSetStateCommand(clawState);

                        CommandBase elevatorSetCommand0 = elevator.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Drop.Cone.elevatorPositionMeters,
                                        Constants.Auto.Arm.Pickup.Drop.Cube.elevatorPositionMeters,
                                        claw::hasCone);

                        CommandBase elbowSetCommand0 = elbow.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Drop.Cone.elbowPositionDegrees,
                                        Constants.Auto.Arm.Pickup.Drop.Cube.elbowPositionDegrees,
                                        claw::hasCone);

                        CommandBase grabberSetCommand0 = grabber.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Drop.Cone.grabberSpeedRPM,
                                        Constants.Auto.Arm.Pickup.Drop.Cube.grabberSpeedRPM,
                                        claw::hasCone);

                        CommandBase elevatorTiltSetCommand0 = tilt.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Drop.Cone.elevatorTiltState,
                                        Constants.Auto.Arm.Pickup.Drop.Cube.elevatorTiltState,
                                        claw::hasCone);

                        BooleanSupplier elbowAndElevatorStopCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return elevator.atSetpoint() && elbow.atSetpoint();
                                }

                        };

                        CommandBase elbowAndElevatorDeadline = Commands.waitUntil(elbowAndElevatorStopCondition);

                        CommandBase Pickup = Commands.deadline(
                                        elbowAndElevatorDeadline,
                                        elevatorSetCommand0,
                                        elbowSetCommand0,
                                        grabberSetCommand0,
                                        elevatorTiltSetCommand0);

                        return Commands.sequence(clawSetCommand0, Pickup);
                }

                public static CommandBase createSliding(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                Claw claw,
                                ElevatorTilt tilt,
                                Claw.State clawState) {

                        CommandBase clawSetCommand0 = claw.createSetStateCommand(clawState);

                        CommandBase elevatorSetCommand0 = elevator.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Sliding.Cone.elevatorPositionMeters,
                                        Constants.Auto.Arm.Pickup.Sliding.Cube.elevatorPositionMeters,
                                        claw::hasCone);

                        CommandBase elbowSetCommand0 = elbow.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Sliding.Cone.elbowPositionDegrees,
                                        Constants.Auto.Arm.Pickup.Sliding.Cube.elbowPositionDegrees,
                                        claw::hasCone);

                        CommandBase grabberSetCommand0 = grabber.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Sliding.Cone.grabberSpeedRPM,
                                        Constants.Auto.Arm.Pickup.Sliding.Cube.grabberSpeedRPM,
                                        claw::hasCone);

                        CommandBase elevatorTiltSetCommand0 = tilt.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Sliding.Cone.elevatorTiltState,
                                        Constants.Auto.Arm.Pickup.Sliding.Cube.elevatorTiltState,
                                        claw::hasCone);

                        BooleanSupplier elbowAndElevatorStopCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return elevator.atSetpoint() && elbow.atSetpoint();
                                }

                        };

                        CommandBase elbowAndElevatorDeadline = Commands.waitUntil(elbowAndElevatorStopCondition);

                        CommandBase Pickup = Commands.deadline(
                                        elbowAndElevatorDeadline,
                                        elevatorSetCommand0,
                                        elbowSetCommand0,
                                        grabberSetCommand0,
                                        elevatorTiltSetCommand0);

                        return Commands.sequence(clawSetCommand0, Pickup);
                }

                public static CommandBase createFloor(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                Claw claw,
                                ElevatorTilt tilt,
                                Claw.State clawState) {

                        CommandBase clawSetCommand0 = claw.createSetStateCommand(clawState);

                        CommandBase elevatorSetCommand0 = elevator.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Floor.Cone.elevatorPositionMeters,
                                        Constants.Auto.Arm.Pickup.Floor.Cube.elevatorPositionMeters,
                                        claw::hasCone);

                        CommandBase elbowSetCommand0 = elbow.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Floor.Cone.elbowPositionDegrees,
                                        Constants.Auto.Arm.Pickup.Floor.Cube.elbowPositionDegrees,
                                        claw::hasCone);

                        CommandBase grabberSetCommand0 = grabber.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Floor.Cone.grabberSpeedRPM,
                                        Constants.Auto.Arm.Pickup.Floor.Cube.grabberSpeedRPM,
                                        claw::hasCone);

                        CommandBase elevatorTiltSetCommand0 = tilt.createConeCubeCommand(
                                        Constants.Auto.Arm.Pickup.Floor.Cone.elevatorTiltState,
                                        Constants.Auto.Arm.Pickup.Floor.Cube.elevatorTiltState,
                                        claw::hasCone);

                        BooleanSupplier elbowAndElevatorStopCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return elevator.atSetpoint() && elbow.atSetpoint();
                                }

                        };

                        CommandBase elbowAndElevatorDeadline = Commands.waitUntil(elbowAndElevatorStopCondition);

                        CommandBase Pickup = Commands.deadline(
                                        elbowAndElevatorDeadline,
                                        elevatorSetCommand0,
                                        elbowSetCommand0,
                                        grabberSetCommand0,
                                        elevatorTiltSetCommand0);

                        return Commands.sequence(clawSetCommand0, Pickup);
                }
        }

        public static class Carry {

                public static CommandBase create(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                ElevatorTilt tilt,
                                Claw claw) {

                        CommandBase elevatorSetCommand0 = elevator.createConeCubeCommand(
                                        Constants.Auto.Arm.Carry.Cone.elevatorPositionMeters,
                                        Constants.Auto.Arm.Carry.Cube.elevatorPositionMeters,
                                        claw::hasCone);

                        CommandBase elbowSetCommand0 = elbow.createConeCubeCommand(
                                        Constants.Auto.Arm.Carry.Cone.elbowPositionDegrees,
                                        Constants.Auto.Arm.Carry.Cube.elbowPositionDegrees,
                                        claw::hasCone);

                        CommandBase grabberSetCommand0 = grabber.createConeCubeCommand(
                                        Constants.Auto.Arm.Carry.Cone.grabberSpeedRPM,
                                        Constants.Auto.Arm.Carry.Cube.grabberSpeedRPM,
                                        claw::hasCone);

                        CommandBase elevatorTiltSetCommand0 = tilt.createConeCubeCommand(
                                        Constants.Auto.Arm.Carry.Cone.elevatorTiltState,
                                        Constants.Auto.Arm.Carry.Cube.elevatorTiltState,
                                        claw::hasCone);

                        BooleanSupplier elbowAndElevatorStopCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return elevator.atSetpoint() && elbow.atSetpoint();
                                }

                        };

                        CommandBase elbowAndElevatorDeadline = Commands.waitUntil(elbowAndElevatorStopCondition);

                        return Commands.deadline(
                                        elbowAndElevatorDeadline,
                                        elevatorSetCommand0,
                                        elbowSetCommand0,
                                        grabberSetCommand0,
                                        elevatorTiltSetCommand0);
                }
        }

}
