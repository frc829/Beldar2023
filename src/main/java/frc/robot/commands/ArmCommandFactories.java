// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
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
                                ElevatorTilt tilt,
                                Claw claw) {

                        BooleanSupplier coneOrCubeCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == Claw.State.CONE;
                                }
                        };

                        CommandBase elevatorSetCommand = elevator.createControlCommand(
                                        Constants.Auto.Arm.Alignment.High.elevatorPositionMeters);

                        CommandBase elbowConeCommand = elbow.createControlCommand(
                                        Constants.Auto.Arm.Alignment.High.elbowPositionDegrees);

                        CommandBase elbowCubeCommand = elbow.createControlCommand(
                                        Constants.Auto.Arm.Alignment.High.elbowPositionCubeDegrees);

                        CommandBase elbowCommand = Commands.either(elbowConeCommand, elbowCubeCommand,
                                        coneOrCubeCondition);

                        CommandBase tiltElevatorCommand = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Alignment.High.elevatorTiltState);
                                        

                        return Commands.parallel(
                                        elevatorSetCommand,
                                        elbowCommand,
                                        tiltElevatorCommand);
                }

                public static CommandBase createMiddle(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw) {

                        BooleanSupplier coneOrCubeCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == Claw.State.CONE;
                                }
                        };

                        CommandBase elevatorSetCommand = elevator.createControlCommand(
                                        Constants.Auto.Arm.Alignment.Middle.elevatorPositionMeters);

                        CommandBase elbowConeCommand = elbow.createControlCommand(
                                        Constants.Auto.Arm.Alignment.Middle.elbowPositionDegrees);

                        CommandBase elbowCubeCommand = elbow.createControlCommand(
                                        Constants.Auto.Arm.Alignment.Middle.elbowPositionCubeDegrees);

                        CommandBase elbowCommand = Commands.either(elbowConeCommand, elbowCubeCommand,
                                        coneOrCubeCondition);

                        CommandBase tiltElevatorCommand = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Alignment.Middle.elevatorTiltState);

                        return Commands.parallel(
                                        elevatorSetCommand,
                                        elbowCommand,
                                        tiltElevatorCommand);
                }

                public static CommandBase createLow(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw) {

                        BooleanSupplier coneOrCubeCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == Claw.State.CONE;
                                }
                        };

                        CommandBase elevatorSetCommand = elevator.createControlCommand(
                                        Constants.Auto.Arm.Alignment.Low.elevatorPositionMeters);

                        CommandBase elbowConeCommand = elbow.createControlCommand(
                                        Constants.Auto.Arm.Alignment.Low.elbowPositionDegrees);

                        CommandBase elbowCubeCommand = elbow.createControlCommand(
                                        Constants.Auto.Arm.Alignment.Low.elbowPositionCubeDegrees);

                        CommandBase elbowCommand = Commands.either(elbowConeCommand, elbowCubeCommand,
                                        coneOrCubeCondition);

                        CommandBase tiltElevatorCommand = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Alignment.Low.elevatorTiltState);

                        CommandBase alignmentDeadline = Commands.waitSeconds(2);

                        CommandBase setArm = Commands.parallel(
                                        elevatorSetCommand,
                                        elbowCommand,
                                        tiltElevatorCommand);

                        return Commands.deadline(alignmentDeadline, setArm);
                }

        }

        public static class Placement {

                public static CommandBase createHigh(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw,
                                Grabber grabber) {

                        BooleanSupplier coneOrCubeCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == Claw.State.CONE;
                                }
                        };

                        CommandBase tiltElevatorCone = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.High.Cone.elevatorTiltState);
                        CommandBase tiltElevatorCube = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.High.Cube.elevatorTiltState);

                        CommandBase tiltElevator = Commands.either(tiltElevatorCone, tiltElevatorCube, coneOrCubeCondition);

                        CommandBase waitForTilt = Commands.waitSeconds(1.5);

                        CommandBase openClaw = claw.createSetStateCommand(Claw.State.CUBE);
                        CommandBase runGrabber = grabber.createControlCommand(-916);

                        CommandBase releaseChoice = Commands.either(openClaw, runGrabber, coneOrCubeCondition);
                        CommandBase releaseDeadline = Commands.waitSeconds(0.5);
                        CommandBase release = Commands.deadline(releaseDeadline, releaseChoice);


                        CommandBase elbowUpCommand = elbow.createControlCommand(15);
                        CommandBase elbowUpDeadline = Commands.waitSeconds(0.5);
                        CommandBase elbowUp = Commands.deadline(elbowUpDeadline, elbowUpCommand);


                        CommandBase tiltBack = tilt.createSetStateCommand(ElevatorTilt.State.NONE);
                        CommandBase elevatorDown = elevator.createControlCommand(0);
                        CommandBase grabberOff = grabber.createStopCommand();
                        
                        CommandBase elbowAdjust = elbow.createControlCommand(35);
                        CommandBase waitForAdjust = Commands.waitSeconds(0.25);
                        CommandBase waitThenAdjustElbow = Commands.sequence(waitForAdjust, elbowAdjust);
                        
                        CommandBase tiltBackElevatorDown = Commands.parallel(waitThenAdjustElbow, tiltBack, elevatorDown, grabberOff);
                        


                        return Commands.sequence(
                                tiltElevator, 
                                waitForTilt,
                                release,
                                //elbowUp,
                                tiltBackElevatorDown);




                }

                public static Command createMiddle(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw,
                                Grabber grabber) {

                        BooleanSupplier coneOrCubeCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == Claw.State.CONE;
                                }
                        };

                        CommandBase tiltElevatorCone = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.Middle.Cone.elevatorTiltState);
                        CommandBase tiltElevatorCube = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.Middle.Cube.elevatorTiltState);

                        CommandBase tiltElevator = Commands.either(tiltElevatorCone, tiltElevatorCube, coneOrCubeCondition);

                        CommandBase waitForTilt = Commands.waitSeconds(1.0);

                        CommandBase openClaw = claw.createSetStateCommand(Claw.State.CUBE);
                        CommandBase runGrabber = grabber.createControlCommand(-200);

                        CommandBase releaseChoice = Commands.either(openClaw, runGrabber, coneOrCubeCondition);
                        CommandBase releaseDeadline = Commands.waitSeconds(0.5);
                        CommandBase release = Commands.deadline(releaseDeadline, releaseChoice);


                        CommandBase elbowUpCommand = elbow.createControlCommand(15);
                        CommandBase elbowUpDeadline = Commands.waitSeconds(0.5);
                        CommandBase elbowUp = Commands.deadline(elbowUpDeadline, elbowUpCommand);


                        CommandBase tiltBack = tilt.createSetStateCommand(ElevatorTilt.State.NONE);
                        CommandBase elevatorDown = elevator.createControlCommand(0);
                        CommandBase grabberOff = grabber.createStopCommand();
                        CommandBase tiltBackElevatorDown = Commands.parallel(tiltBack, elevatorDown, grabberOff);
                        

                        return Commands.sequence(
                                tiltElevator, 
                                waitForTilt,
                                release,
                                //elbowUp,
                                tiltBackElevatorDown);




                }

                public static Command createLow(
                                Elevator elevator,
                                Elbow elbow,
                                ElevatorTilt tilt,
                                Claw claw,
                                Grabber grabber) {

                        BooleanSupplier coneOrCubeCondition = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == Claw.State.CONE;
                                }
                        };

                        CommandBase tiltElevatorCone = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.Low.Cone.elevatorTiltState);
                        CommandBase tiltElevatorCube = tilt.createSetStateCommand(
                                        Constants.Auto.Arm.Placement.Low.Cube.elevatorTiltState);

                        CommandBase tiltElevator = Commands.either(tiltElevatorCone, tiltElevatorCube, coneOrCubeCondition);

                        CommandBase waitForTilt = Commands.waitSeconds(0.5);

                        CommandBase openClaw = claw.createSetStateCommand(Claw.State.CUBE);
                        CommandBase runGrabber = grabber.createControlCommand(-100);

                        CommandBase releaseChoice = Commands.either(openClaw, runGrabber, coneOrCubeCondition);
                        CommandBase releaseDeadline = Commands.waitSeconds(0.5);
                        CommandBase release = Commands.deadline(releaseDeadline, releaseChoice);


                        CommandBase elbowUpCommand = elbow.createControlCommand(15);
                        CommandBase elbowUpDeadline = Commands.waitSeconds(0.5);
                        CommandBase elbowUp = Commands.deadline(elbowUpDeadline, elbowUpCommand);


                        CommandBase tiltBack = tilt.createSetStateCommand(ElevatorTilt.State.NONE);
                        CommandBase elevatorDown = elevator.createControlCommand(0);
                        CommandBase grabberOff = grabber.createStopCommand();
                        CommandBase tiltBackElevatorDown = Commands.parallel(tiltBack, elevatorDown, grabberOff);
                        

                        return Commands.sequence(
                                tiltElevator, 
                                waitForTilt,
                                release,
                                //elbowUp,
                                tiltBackElevatorDown);




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

                        BooleanSupplier coneOrCube = new BooleanSupplier() {

                                @Override
                                public boolean getAsBoolean() {
                                        return claw.getState() == Claw.State.CONE;
                                }

                        };

                        CommandBase elevatorSetCommand = elevator.createControlCommand(elevatorPositionMeters);
                        CommandBase elbowSetCommand = elbow.createControlCommand(elbowPositionDegrees);

                        CommandBase coneGrabberSet = grabber.createControlCommand(grabberSpeedRPM);
                        CommandBase cubeGrabberSEt = grabber.createControlCommand(500);

                        CommandBase grabberSetCommand = Commands.either(coneGrabberSet, cubeGrabberSEt, coneOrCube);
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
