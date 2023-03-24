// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Telemetry;

/** Add your docs here. */
public abstract class PathPlannerToAuto {

    public static List<PathPlannerTrajectory> getPathPlannerTrajectory(
            String pathName,
            PathConstraints firstPathConstraint,
            PathConstraints... remainingPathConstraints) {

        return PathPlanner.loadPathGroup(
                pathName,
                firstPathConstraint,
                remainingPathConstraints);
    }

    public static List<Pose2d> getTrajectoryPoses(
            List<PathPlannerTrajectory> pathPlannerTrajectories) {
        List<Pose2d> poses = new ArrayList<Pose2d>();

        for (PathPlannerTrajectory pathPlannerTrajectory : pathPlannerTrajectories) {
            PathPlannerTrajectory transformedTrajectory = PathPlannerTrajectory
                    .transformTrajectoryForAlliance(pathPlannerTrajectory, DriverStation.getAlliance());
            for (State pathPoint : transformedTrajectory.getStates()) {
                poses.add(pathPoint.poseMeters);
            }
        }

        return poses;
    }

    public static Command createFullAutoFromPathGroup(
            SwerveDrive swerveDrive,
            Telemetry telemetry,
            Arm arm,
            List<PathPlannerTrajectory> pathGroup,
            PIDConstants translationConstants,
            PIDConstants rotationConstants) {

        HashMap<String, Command> eventMap = new HashMap<>();

        List<String> commandNames = new ArrayList<>();

        for (PathPlannerTrajectory pathPlannerTrajectory : pathGroup) {
            List<EventMarker> eventMarkers = pathPlannerTrajectory.getMarkers();
            StopEvent startStopEvent = pathPlannerTrajectory.getStartStopEvent();
            StopEvent endStopEvent = pathPlannerTrajectory.getEndStopEvent();

            for (String name : startStopEvent.names) {
                commandNames.add(name);
            }
            for (String name : endStopEvent.names) {
                commandNames.add(name);
            }

            for (EventMarker eventMarker : eventMarkers) {
                for (String name : eventMarker.names) {
                    commandNames.add(name);
                }
            }
        }

        for (String name : commandNames) {
            CommandBase command = createCommandForAuto(
                name, 
                swerveDrive, 
                arm,
                telemetry);
            eventMap.put(name, command);
        }

        return createFullAuto(
                swerveDrive,
                telemetry,
                eventMap,
                pathGroup,
                translationConstants,
                rotationConstants);
    }

    private static CommandBase createCommandForAuto(
        String name, 
        SwerveDrive swerveDrive, 
        Arm arm,
        Telemetry telemetry) {
        if (name.contains("ScoreHighCone")) {
            CommandBase clawCone = claw.createSetStateCommand(Claw.State.CONE);
            CommandBase alignment = ArmCommands.createAlignHigh(elevator, elbow, tilt, claw);
            CommandBase placement = ArmCommands.createHighPlacement(elevator, elbow, tilt, claw, grabber);
            CommandBase reset = ArmCommands.createResetHigh(elevator, elbow, tilt, grabber);
            return Commands.sequence(clawCone, alignment, placement, reset);
        } else if (name.contains("ScoreHighCube")) {
            CommandBase clawCube = claw.createSetStateCommand(Claw.State.CUBE);
            CommandBase alignment = ArmCommands.createAlignHigh(elevator, elbow, tilt, claw);
            CommandBase placement = ArmCommands.createHighPlacement(elevator, elbow, tilt, claw, grabber);
            CommandBase reset = ArmCommands.createResetHigh(elevator, elbow, tilt, grabber);
            return Commands.sequence(clawCube, alignment, placement, reset);
        } else if (name.contains("ScoreMiddleCone")) {
            CommandBase clawCone = claw.createSetStateCommand(Claw.State.CONE);
            CommandBase alignment = ArmCommands.createAlignMiddle(elevator, elbow, tilt, claw);
            CommandBase placement = ArmCommands.createMiddlePlacement(elevator, elbow, tilt, claw, grabber);
            CommandBase reset = ArmCommands.createResetMiddle(elevator, elbow, tilt, grabber);
            return Commands.sequence(clawCone, alignment, placement, reset);
        } else if (name.contains("ScoreMiddleCube")) {
            CommandBase clawCube = claw.createSetStateCommand(Claw.State.CUBE);
            CommandBase alignment = ArmCommands.createAlignMiddle(elevator, elbow, tilt, claw);
            CommandBase placement = ArmCommands.createMiddlePlacement(elevator, elbow, tilt, claw, grabber);
            CommandBase reset = ArmCommands.createResetMiddle(elevator, elbow, tilt, grabber);
            return Commands.sequence(clawCube, alignment, placement, reset);
        } else if (name.contains("ScoreLow")) {
            CommandBase alignment = ArmCommands.createAlignLow(elevator, elbow, tilt, claw);
            CommandBase placement = ArmCommands.createLowPlacement(elevator, elbow, tilt, claw, grabber);
            CommandBase reset = ArmCommands.createResetLow(elevator, elbow, tilt, grabber);
            return Commands.sequence(alignment, placement, reset);
        } else if (name.contains("ConePickup")) {
            CommandBase conePickup = ArmCommands.createFloorPickup(
                    elevator,
                    elbow,
                    tilt,
                    grabber,
                    claw,
                    ledLighting,
                    Claw.State.CONE);
            return conePickup;
        } else if (name.contains("CubePickup")) {
            CommandBase conePickup = ArmCommands.createFloorPickup(
                    elevator,
                    elbow,
                    tilt,
                    grabber,
                    claw,
                    ledLighting,
                    Claw.State.CUBE);
            return conePickup;
        } else if (name.contains("ElevatorHold")) {
            return elevator.createHoldCommand();
        } else if (name.contains("ElbowHold")) {
            return elbow.createHoldCommand();
        } else if (name.contains("Carry")) {
            CommandBase carry = ArmCommands.createCarry(
                    elevator,
                    elbow,
                    tilt,
                    claw);
            return carry;
        } else if (name.contains("Camera")) {
            CommandBase camera = telemetry.setTelemetryFromCameraCommand();
            return camera;
        } else {
            CommandBase none = Commands.none();
            return none;
        }
    }

    private static Command createFullAuto(
            SwerveDrive swerveDrive,
            Telemetry telemetry,
            Map<String, Command> eventMap,
            List<PathPlannerTrajectory> pathGroup,
            PIDConstants translationConstants,
            PIDConstants rotationConstants) {

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                telemetry::getSwerveDrivePosition,
                telemetry::resetSwerveDrivePosition,
                translationConstants,
                rotationConstants,
                swerveDrive::setSwerveDriveChassisSpeed,
                eventMap,
                true,
                swerveDrive,
                telemetry);

        Command fullAuto = autoBuilder.fullAuto(pathGroup);

        return fullAuto;

    }

}
