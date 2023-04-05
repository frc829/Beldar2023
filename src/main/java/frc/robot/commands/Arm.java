// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.framework.lighting.LEDConfig;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTilt;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.LEDLighting;
import frc.robot.subsystems.ElevatorTilt.State;

/** Add your docs here. */
public abstract class Arm {

        public static CommandBase createAlignHigh(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw) {

                CommandBase movement0 = Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                claw,
                                Constants.Auto.Arm.Alignment.High.Cone.elevatorTiltState0,
                                Constants.Auto.Arm.Alignment.High.Cube.elevatorTiltState0,
                                Constants.Auto.Arm.Alignment.High.Cone.elevatorPositionMeters0,
                                Constants.Auto.Arm.Alignment.High.Cube.elevatorPositionMeters0,
                                Constants.Auto.Arm.Alignment.High.Cone.elbowPositionDegrees0,
                                Constants.Auto.Arm.Alignment.High.Cube.elbowPositionDegrees0);

                CommandBase movement1 = Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                claw,
                                Constants.Auto.Arm.Alignment.High.Cone.elevatorTiltState1,
                                Constants.Auto.Arm.Alignment.High.Cube.elevatorTiltState1,
                                Constants.Auto.Arm.Alignment.High.Cone.elevatorPositionMeters1,
                                Constants.Auto.Arm.Alignment.High.Cube.elevatorPositionMeters1,
                                Constants.Auto.Arm.Alignment.High.Cone.elbowPositionDegrees1,
                                Constants.Auto.Arm.Alignment.High.Cube.elbowPositionDegrees1);

                return Commands.sequence(movement0, movement1);
        }

        public static CommandBase createAlignMiddle(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw) {
                return Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                claw,
                                Constants.Auto.Arm.Alignment.Middle.Cone.elevatorTiltState,
                                Constants.Auto.Arm.Alignment.Middle.Cube.elevatorTiltState,
                                Constants.Auto.Arm.Alignment.Middle.Cone.elevatorPositionMeters,
                                Constants.Auto.Arm.Alignment.Middle.Cube.elevatorPositionMeters,
                                Constants.Auto.Arm.Alignment.Middle.Cone.elbowPositionDegrees,
                                Constants.Auto.Arm.Alignment.Middle.Cube.elbowPositionDegrees);
        }

        public static CommandBase createAlignLow(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw) {
                return Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                claw,
                                Constants.Auto.Arm.Alignment.Low.Cone.elevatorTiltState,
                                Constants.Auto.Arm.Alignment.Low.Cube.elevatorTiltState,
                                Constants.Auto.Arm.Alignment.Low.Cone.elevatorPositionMeters,
                                Constants.Auto.Arm.Alignment.Low.Cube.elevatorPositionMeters,
                                Constants.Auto.Arm.Alignment.Low.Cone.elbowPositionDegrees,
                                Constants.Auto.Arm.Alignment.Low.Cube.elbowPositionDegrees);
        }

        public static CommandBase createHighConePoof(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw,
                        Grabber grabber) {

                CommandBase movement0 = Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                claw,
                                Constants.Auto.Arm.Alignment.High.Cone.elevatorTiltState0,
                                Constants.Auto.Arm.Alignment.High.Cube.elevatorTiltState0,
                                Constants.Auto.Arm.Alignment.High.Cone.elevatorPositionMeters0,
                                Constants.Auto.Arm.Alignment.High.Cube.elevatorPositionMeters0,
                                Constants.Auto.Arm.Alignment.High.Cone.elbowPositionDegrees0,
                                Constants.Auto.Arm.Alignment.High.Cube.elbowPositionDegrees0);

                CommandBase movement1 = Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                claw,
                                Constants.Auto.Arm.Alignment.High.Cone.elevatorTiltState1,
                                Constants.Auto.Arm.Alignment.High.Cube.elevatorTiltState1,
                                Constants.Auto.Arm.Alignment.High.Cone.elevatorPositionMeters1,
                                Constants.Auto.Arm.Alignment.High.Cube.elevatorPositionMeters1,
                                Constants.Auto.Arm.Alignment.High.Cone.elbowPositionDegrees1,
                                Constants.Auto.Arm.Alignment.High.Cube.elbowPositionDegrees1);

                CommandBase tinyWait = Commands.waitSeconds(0.25);

                CommandBase openClaw = claw.createSetStateCommand(Claw.State.CUBE);

                return Commands.sequence(movement0, movement1, tinyWait, openClaw);
        }

        public static CommandBase createHighPlacement(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw,
                        Grabber grabber) {

                CommandBase movement0 = Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                claw,
                                Constants.Auto.Arm.Placement.High.Cone.elevatorTiltState,
                                Constants.Auto.Arm.Placement.High.Cube.elevatorTiltState);

                CommandBase wait0 = Arm.SingleArmMovement.ArmWaiting(
                                claw,
                                Constants.Auto.Arm.Placement.High.Cone.waitForElevatorToTilt,
                                Constants.Auto.Arm.Placement.High.Cube.waitForElevatorToTilt);

                CommandBase movement1 = Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                claw,
                                grabber,
                                Constants.Auto.Arm.Placement.High.Cube.grabberSpeedCubeRPM,
                                0.25);

                CommandBase tinyWait = Commands.waitSeconds(0.25);

                return Commands.sequence(movement0, wait0, movement1, tinyWait);
        }

        public static CommandBase createMiddlePlacement(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw,
                        Grabber grabber) {

                CommandBase movement0 = Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                claw,
                                Constants.Auto.Arm.Placement.Middle.Cone.elevatorTiltState,
                                Constants.Auto.Arm.Placement.Middle.Cube.elevatorTiltState);

                CommandBase wait0 = Arm.SingleArmMovement.ArmWaiting(
                                claw,
                                Constants.Auto.Arm.Placement.Middle.Cone.waitForElevatorToTilt,
                                Constants.Auto.Arm.Placement.Middle.Cube.waitForElevatorToTilt);

                CommandBase movement1 = Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                claw,
                                grabber,
                                Constants.Auto.Arm.Placement.Middle.Cube.grabberSpeedCubeRPM,
                                0.25);

                CommandBase tinyWait = Commands.waitSeconds(0.25);

                return Commands.sequence(movement0, wait0, movement1, tinyWait);
        }

        public static CommandBase createLowPlacement(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw,
                        Grabber grabber) {

                CommandBase movement0 = Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                claw,
                                Constants.Auto.Arm.Placement.Low.Cone.elevatorTiltState,
                                Constants.Auto.Arm.Placement.Low.Cube.elevatorTiltState);

                CommandBase wait0 = Arm.SingleArmMovement.ArmWaiting(
                                claw,
                                Constants.Auto.Arm.Placement.Low.Cone.waitForElevatorToTilt,
                                Constants.Auto.Arm.Placement.Low.Cube.waitForElevatorToTilt);

                CommandBase movement1 = Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                claw,
                                grabber,
                                Constants.Auto.Arm.Placement.Low.Cube.grabberSpeedCubeRPM,
                                0.25);

                CommandBase tinyWait = Commands.waitSeconds(0.25);

                return Commands.sequence(movement0, wait0, movement1, tinyWait);
        }

        public static CommandBase createFloorPickup(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Grabber grabber,
                        Claw claw,
                        LEDLighting ledLighting,
                        Claw.State clawState) {
                return Arm.SingleArmMovement.create(
                                elevator,
                                elbow,
                                tilt,
                                grabber,
                                claw,
                                ledLighting,
                                clawState,
                                Constants.Auto.Arm.Pickup.Floor.Cone.elevatorTiltState,
                                Constants.Auto.Arm.Pickup.Floor.Cube.elevatorTiltState,
                                Constants.Auto.Arm.Pickup.Floor.Cone.elevatorPositionMeters,
                                Constants.Auto.Arm.Pickup.Floor.Cube.elevatorPositionMeters,
                                Constants.Auto.Arm.Pickup.Floor.Cone.elbowPositionDegrees,
                                Constants.Auto.Arm.Pickup.Floor.Cube.elbowPositionDegrees,
                                Constants.Auto.Arm.Pickup.Floor.Cone.grabberSpeedRPM,
                                Constants.Auto.Arm.Pickup.Floor.Cube.grabberSpeedRPM);
        }

        public static CommandBase createPoof(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Grabber grabber,
                        Claw claw,
                        LEDLighting ledLighting,
                        Claw.State clawState) {
                return Arm.SingleArmMovement.create(
                                elevator,
                                elbow,
                                tilt,
                                grabber,
                                claw,
                                ledLighting,
                                clawState,
                                Constants.Auto.Arm.Pickup.Floor.Cone.elevatorTiltState,
                                Constants.Auto.Arm.Pickup.Floor.Cube.elevatorTiltState,
                                Constants.Auto.Arm.Pickup.Floor.Cone.elevatorPositionMeters,
                                Constants.Auto.Arm.Pickup.Floor.Cube.elevatorPositionMeters,
                                Constants.Auto.Arm.Pickup.Floor.Cone.elbowPositionDegrees,
                                Constants.Auto.Arm.Pickup.Floor.Cube.elbowPositionDegrees,
                                -916,
                                -160);
        }

        public static CommandBase createSlidingPickup(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Grabber grabber,
                        Claw claw,
                        LEDLighting ledLighting,
                        Claw.State clawState) {
                return Arm.SingleArmMovement.create(
                                elevator,
                                elbow,
                                tilt,
                                grabber,
                                claw,
                                ledLighting,
                                clawState,
                                Constants.Auto.Arm.Pickup.Sliding.Cone.elevatorTiltState,
                                Constants.Auto.Arm.Pickup.Sliding.Cube.elevatorTiltState,
                                Constants.Auto.Arm.Pickup.Sliding.Cone.elevatorPositionMeters,
                                Constants.Auto.Arm.Pickup.Sliding.Cube.elevatorPositionMeters,
                                Constants.Auto.Arm.Pickup.Sliding.Cone.elbowPositionDegrees,
                                Constants.Auto.Arm.Pickup.Sliding.Cube.elbowPositionDegrees,
                                Constants.Auto.Arm.Pickup.Sliding.Cone.grabberSpeedRPM,
                                Constants.Auto.Arm.Pickup.Sliding.Cube.grabberSpeedRPM);
        }

        public static CommandBase createDropPickup(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Grabber grabber,
                        Claw claw,
                        LEDLighting ledLighting,
                        Claw.State clawState) {
                return Arm.SingleArmMovement.create(
                                elevator,
                                elbow,
                                tilt,
                                grabber,
                                claw,
                                ledLighting,
                                clawState,
                                Constants.Auto.Arm.Pickup.Drop.Cone.elevatorTiltState,
                                Constants.Auto.Arm.Pickup.Drop.Cube.elevatorTiltState,
                                Constants.Auto.Arm.Pickup.Drop.Cone.elevatorPositionMeters,
                                Constants.Auto.Arm.Pickup.Drop.Cube.elevatorPositionMeters,
                                Constants.Auto.Arm.Pickup.Drop.Cone.elbowPositionDegrees,
                                Constants.Auto.Arm.Pickup.Drop.Cube.elbowPositionDegrees,
                                Constants.Auto.Arm.Pickup.Drop.Cone.grabberSpeedRPM,
                                Constants.Auto.Arm.Pickup.Drop.Cube.grabberSpeedRPM);
        }

        public static CommandBase createResetHigh(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Grabber grabber) {
                return Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                grabber,
                                Constants.Auto.Arm.Reset.High.elevatorTiltState,
                                Constants.Auto.Arm.Reset.High.elevatorPosition,
                                Constants.Auto.Arm.Reset.High.elbowPosition,
                                Constants.Auto.Arm.Reset.High.grabberSpeed);
        }

        public static CommandBase createResetMiddle(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Grabber grabber) {
                return Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                grabber,
                                Constants.Auto.Arm.Reset.Middle.elevatorTiltState,
                                Constants.Auto.Arm.Reset.Middle.elevatorPosition,
                                Constants.Auto.Arm.Reset.Middle.elbowPosition,
                                Constants.Auto.Arm.Reset.Middle.grabberSpeed);
        }

        public static CommandBase createResetLow(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Grabber grabber) {
                return Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                grabber,
                                Constants.Auto.Arm.Reset.Low.elevatorTiltState,
                                Constants.Auto.Arm.Reset.Low.elevatorPosition,
                                Constants.Auto.Arm.Reset.Low.elbowPosition,
                                Constants.Auto.Arm.Reset.Low.grabberSpeed);
        }

        public static CommandBase createCarry(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw) {
                return Arm.SingleArmMovement.createWithEnding(
                                elevator,
                                elbow,
                                tilt,
                                claw,
                                Constants.Auto.Arm.Carry.Cone.elevatorTiltState,
                                Constants.Auto.Arm.Carry.Cube.elevatorTiltState,
                                Constants.Auto.Arm.Carry.Cone.elevatorPositionMeters,
                                Constants.Auto.Arm.Carry.Cube.elevatorPositionMeters,
                                Constants.Auto.Arm.Carry.Cone.elbowPositionDegrees,
                                Constants.Auto.Arm.Carry.Cube.elbowPositionDegrees);
        }

        public static CommandBase createHighConePoofAndReset(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw,
                        Grabber grabber) {
                return Commands.sequence(
                                Arm.createHighConePoof(elevator, elbow, tilt, claw, grabber),
                                Arm.createResetHigh(elevator, elbow, tilt, grabber));
        }

        public static CommandBase createHighPlacementAndReset(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw,
                        Grabber grabber) {

                return Commands.sequence(
                                Arm.createHighPlacement(elevator, elbow, tilt, claw,
                                                grabber),
                                Arm.createResetHigh(elevator, elbow, tilt, grabber));
        }

        public static CommandBase createMiddlePlacementAndReset(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw,
                        Grabber grabber) {

                return Commands.sequence(
                                Arm.createMiddlePlacement(elevator, elbow, tilt, claw,
                                                grabber),
                                Arm.createResetMiddle(elevator, elbow, tilt, grabber));
        }

        public static CommandBase createLowAlignPlacementAndReset(
                        Elevator elevator,
                        Elbow elbow,
                        ElevatorTilt tilt,
                        Claw claw,
                        Grabber grabber) {

                return Commands.sequence(
                                Arm.createAlignLow(elevator, elbow, tilt, claw),
                                Arm.createLowPlacement(elevator, elbow, tilt, claw,
                                                grabber),
                                Arm.createResetLow(elevator, elbow, tilt,
                                                grabber));

        }

        public static class SingleArmMovement {

                public static CommandBase createToggleClawAndLights(
                                Claw claw,
                                LEDLighting ledLighting) {

                        CommandBase toggleClawAndLightsCommand = new CommandBase() {
                                @Override
                                public void initialize() {
                                        if (claw.getState() == Claw.State.CONE) {
                                                claw.setState(Claw.State.CUBE);
                                                LEDConfig cubeLEDConfig = new LEDConfig(
                                                                Constants.Auto.Arm.LEDLighting.Cube, 0, 400);
                                                ledLighting.setCurrentLEDConfig(cubeLEDConfig);
                                        } else {
                                                claw.setState(Claw.State.CONE);
                                                LEDConfig coneLEDConfig = new LEDConfig(
                                                                Constants.Auto.Arm.LEDLighting.Cone, 0, 400);
                                                ledLighting.setCurrentLEDConfig(coneLEDConfig);
                                        }
                                }

                                @Override
                                public boolean isFinished() {
                                        return true;
                                }
                        };

                        toggleClawAndLightsCommand.addRequirements(claw, ledLighting);
                        return toggleClawAndLightsCommand;
                }

                public static CommandBase create(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Grabber grabber,
                                Claw claw,
                                LEDLighting ledLighting,
                                Claw.State clawState,
                                ElevatorTilt.State coneTiltState,
                                ElevatorTilt.State cubeTiltState,
                                double elevatorPositionCone,
                                double elevatorPositionCube,
                                double elbowPositionCone,
                                double elbowPositionCube,
                                double grabberSpeedCone,
                                double grabberSpeedCube) {

                        CommandBase singleArmMovementCommand = new CommandBase() {
                                @Override
                                public void initialize() {
                                        if (clawState == Claw.State.CONE) {
                                                claw.setState(Claw.State.CONE);
                                                LEDConfig coneLEDConfig = new LEDConfig(
                                                                Constants.Auto.Arm.LEDLighting.Cone, 0, 400);
                                                ledLighting.setCurrentLEDConfig(coneLEDConfig);
                                                tilt.setState(coneTiltState);
                                                elevator.getPIDController().setSetpoint(elevatorPositionCone);
                                                elbow.getPIDController().setSetpoint(elbowPositionCone / 360.0);
                                        } else {
                                                claw.setState(Claw.State.CUBE);
                                                LEDConfig cubeLEDConfig = new LEDConfig(
                                                                Constants.Auto.Arm.LEDLighting.Cube, 0, 400);
                                                ledLighting.setCurrentLEDConfig(cubeLEDConfig);
                                                tilt.setState(cubeTiltState);
                                                elevator.getPIDController().setSetpoint(elevatorPositionCube);
                                                elbow.getPIDController().setSetpoint(elbowPositionCube / 360.0);
                                        }
                                }

                                @Override
                                public void execute() {
                                        if (claw.getState() == Claw.State.CONE) {
                                                grabber.setAverageVelocityPerSecond(grabberSpeedCone);
                                        } else {
                                                grabber.setAverageVelocityPerSecond(grabberSpeedCube);
                                        }
                                        double elevatorPositionMeters = elevator.getPosition();
                                        double elbowPositionRotations = elbow.getPositionRotations();
                                        double elevatorSpeed = elevator.getPIDController()
                                                        .calculate(elevatorPositionMeters);
                                        double elbowSpeed = elbow.getPIDController().calculate(elbowPositionRotations);
                                        elbow.setVelocityRotationsPerSecond(elbowSpeed);
                                        elevator.setVelocity(elevatorSpeed);
                                }
                        };

                        singleArmMovementCommand.addRequirements(elevator, elbow, tilt, grabber, claw, ledLighting);
                        return singleArmMovementCommand;
                }

                public static CommandBase createWithEnding(
                                Elevator elevator,
                                Elbow elbow,
                                Grabber grabber,
                                double grabberSpeedRPM,
                                double grabberReleaseWaitSeconds) {
                        CommandBase singleArmMovementCommand = new CommandBase() {

                                double startTime = Timer.getFPGATimestamp();
                                double currentTime = startTime;

                                @Override
                                public void initialize() {
                                        elevator.getPIDController().setSetpoint(elevator.getPosition());
                                        elbow.getPIDController().setSetpoint(elbow.getPositionRotations());
                                }

                                @Override
                                public void execute() {

                                        grabber.setAverageVelocityPerSecond(grabberSpeedRPM);
                                        currentTime = Timer.getFPGATimestamp();

                                        double elevatorPositionMeters = elevator.getPosition();
                                        double elbowPositionRotations = elbow.getPositionRotations();
                                        double elevatorSpeed = elevator.getPIDController()
                                                        .calculate(elevatorPositionMeters);
                                        double elbowSpeed = elbow.getPIDController().calculate(elbowPositionRotations);
                                        elbow.setVelocityRotationsPerSecond(elbowSpeed);
                                        elevator.setVelocity(elevatorSpeed);
                                }

                                @Override
                                public boolean isFinished() {
                                        return currentTime - startTime >= grabberReleaseWaitSeconds;
                                }
                        };

                        singleArmMovementCommand.addRequirements(elevator, elbow, grabber);
                        return singleArmMovementCommand;
                }

                public static CommandBase createWithEnding(
                                Elevator elevator,
                                Elbow elbow,
                                Claw claw,
                                Grabber grabber,
                                double grabberSpeedCubeRPM,
                                double grabberReleaseWaitSeconds) {

                        CommandBase singleArmMovementCommand = new CommandBase() {
                                double startTime = Timer.getFPGATimestamp();
                                double currentTime = startTime;

                                @Override
                                public void initialize() {
                                        elevator.getPIDController().setSetpoint(elevator.getPosition());
                                        elbow.getPIDController().setSetpoint(elbow.getPositionRotations());
                                }

                                @Override
                                public void execute() {
                                        if (claw.getState() == Claw.State.CONE) {
                                                claw.setState(Claw.State.CUBE);
                                                currentTime = startTime + grabberReleaseWaitSeconds;
                                        } else {
                                                grabber.setAverageVelocityPerSecond(grabberSpeedCubeRPM);
                                                currentTime = Timer.getFPGATimestamp();
                                        }
                                        double elevatorPositionMeters = elevator.getPosition();
                                        double elbowPositionRotations = elbow.getPositionRotations();
                                        double elevatorSpeed = elevator.getPIDController()
                                                        .calculate(elevatorPositionMeters);
                                        double elbowSpeed = elbow.getPIDController().calculate(elbowPositionRotations);
                                        elbow.setVelocityRotationsPerSecond(elbowSpeed);
                                        elevator.setVelocity(elevatorSpeed);
                                }

                                @Override
                                public boolean isFinished() {
                                        return currentTime - startTime >= grabberReleaseWaitSeconds;
                                }
                        };

                        singleArmMovementCommand.addRequirements(elevator, elbow, grabber, claw);
                        return singleArmMovementCommand;
                }

                public static CommandBase createWithEnding(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw,
                                State elevatorTiltStateCone,
                                State elevatorTiltStateCube) {

                        CommandBase singleArmMovementCommand = new CommandBase() {
                                @Override
                                public void initialize() {
                                        if (claw.getState() == Claw.State.CONE) {
                                                tilt.setState(elevatorTiltStateCone);
                                        } else {
                                                tilt.setState(elevatorTiltStateCube);
                                        }
                                        elevator.getPIDController().setSetpoint(elevator.getPosition());
                                        elbow.getPIDController().setSetpoint(elbow.getPositionRotations());
                                }

                                @Override
                                public void execute() {
                                        double elevatorPositionMeters = elevator.getPosition();
                                        double elbowPositionRotations = elbow.getPositionRotations();
                                        double elevatorSpeed = elevator.getPIDController()
                                                        .calculate(elevatorPositionMeters);
                                        double elbowSpeed = elbow.getPIDController().calculate(elbowPositionRotations);
                                        elbow.setVelocityRotationsPerSecond(elbowSpeed);
                                        elevator.setVelocity(elevatorSpeed);
                                }

                                @Override
                                public boolean isFinished() {
                                        if (elevator.atSetpoint() && elbow.atSetpoint()) {
                                                return true;
                                        } else {
                                                return false;
                                        }
                                }
                        };

                        singleArmMovementCommand.addRequirements(elevator, elbow, tilt, claw);
                        return singleArmMovementCommand;
                }

                public static CommandBase createWithEnding(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw,
                                ElevatorTilt.State coneTiltState,
                                ElevatorTilt.State cubeTiltState,
                                double elevatorPositionCone,
                                double elevatorPositionCube,
                                double elbowPositionCone,
                                double elbowPositionCube) {

                        CommandBase singleArmMovementCommand = new CommandBase() {
                                @Override
                                public void initialize() {
                                        if (claw.getState() == Claw.State.CONE) {
                                                tilt.setState(coneTiltState);
                                                elevator.getPIDController().setSetpoint(elevatorPositionCone);
                                                elbow.getPIDController().setSetpoint(elbowPositionCone / 360.0);
                                        } else {
                                                tilt.setState(cubeTiltState);
                                                elevator.getPIDController().setSetpoint(elevatorPositionCube);
                                                elbow.getPIDController().setSetpoint(elbowPositionCube / 360.0);
                                        }
                                }

                                @Override
                                public void execute() {
                                        double elevatorPositionMeters = elevator.getPosition();
                                        double elbowPositionRotations = elbow.getPositionRotations();
                                        double elevatorSpeed = elevator.getPIDController()
                                                        .calculate(elevatorPositionMeters);
                                        double elbowSpeed = elbow.getPIDController().calculate(elbowPositionRotations);
                                        elbow.setVelocityRotationsPerSecond(elbowSpeed);
                                        elevator.setVelocity(elevatorSpeed);
                                }

                                @Override
                                public boolean isFinished() {
                                        if (elevator.atSetpoint() && elbow.atSetpoint()) {
                                                return true;
                                        } else {
                                                return false;
                                        }
                                }
                        };

                        singleArmMovementCommand.addRequirements(elevator, elbow, tilt, claw);
                        return singleArmMovementCommand;
                }

                public static CommandBase createWithEnding(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Grabber grabber,
                                State elevatorTiltState,
                                double elevatorPosition,
                                double elbowPosition,
                                double grabberSpeed) {

                        CommandBase singleArmMovementCommand = new CommandBase() {
                                @Override
                                public void initialize() {
                                        tilt.setState(elevatorTiltState);
                                        elevator.getPIDController().setSetpoint(elevatorPosition);
                                        elbow.getPIDController().setSetpoint(elbowPosition / 360.0);
                                }

                                @Override
                                public void execute() {
                                        grabber.setAverageVelocityPerSecond(grabberSpeed);
                                        double elevatorPositionMeters = elevator.getPosition();
                                        double elbowPositionRotations = elbow.getPositionRotations();
                                        double elevatorSpeed = elevator.getPIDController()
                                                        .calculate(elevatorPositionMeters);
                                        elevatorSpeed = Math.abs(elevatorSpeed) > 0.5 ? Math.signum(elevatorSpeed) * 0.5
                                                        : elevatorSpeed;
                                        double elbowSpeed = elbow.getPIDController().calculate(elbowPositionRotations);
                                        elbow.setVelocityRotationsPerSecond(elbowSpeed);
                                        elevator.setVelocity(elevatorSpeed);
                                }

                                @Override
                                public boolean isFinished() {
                                        if (elevator.atSetpoint() && elbow.atSetpoint()) {
                                                return true;
                                        } else {
                                                return false;
                                        }
                                }
                        };

                        singleArmMovementCommand.addRequirements(elevator, elbow, tilt, grabber);
                        return singleArmMovementCommand;
                }

                public static CommandBase ArmWaiting(
                                Claw claw, double waitTimeSecondsCone, double waitTimeSecondsCube) {

                        CommandBase waitCone = Commands.waitSeconds(waitTimeSecondsCone);
                        CommandBase waitCube = Commands.waitSeconds(waitTimeSecondsCube);
                        return Commands.either(waitCone, waitCube, claw::hasCone);

                }
        }
}