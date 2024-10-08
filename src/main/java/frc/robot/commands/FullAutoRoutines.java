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
import frc.robot.Constants;
import frc.robot.commands.ArmAutoFactories.ElementType;
import frc.robot.commands.ArmAutoFactories.PlacementType;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTilt;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.LEDLighting;
import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class FullAutoRoutines {

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
            Elevator elevator,
            Elbow elbow,
            Grabber grabber,
            Claw claw,
            ElevatorTilt tilt,
            LEDLighting ledLighting,
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
            CommandBase command = createCommandForAuto(name, swerveDrive, elevator, elbow, grabber, claw, tilt,
                    ledLighting);
            eventMap.put(name, command);
        }

        return createFullAuto(
                swerveDrive,
                eventMap,
                pathGroup,
                translationConstants,
                rotationConstants);
    }

    private static Command createFullAuto(
            SwerveDrive swerveDrive,
            Map<String, Command> eventMap,
            List<PathPlannerTrajectory> pathGroup,
            PIDConstants translationConstants,
            PIDConstants rotationConstants) {

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                swerveDrive::getSwerveDrivePosition,
                swerveDrive::resetSwerveDrivePosition,
                translationConstants,
                rotationConstants,
                swerveDrive::setSwerveDriveChassisSpeed,
                eventMap,
                true,
                swerveDrive);

        Command fullAuto = autoBuilder.fullAuto(pathGroup);

        return fullAuto;

    }

    private static CommandBase createCommandForAuto(
            String name,
            SwerveDrive swerveDrive,
            Elevator elevator,
            Elbow elbow,
            Grabber grabber,
            Claw claw,
            ElevatorTilt tilt,
            LEDLighting ledLighting) {
        if (name.contains("Balance")) {
            return swerveDrive.getBalanceCommand();
        } else if (name.contains("ScoreConeHigh")) {
            return ArmAutoFactories.AlignmentAndPlacement.create(
                    elevator,
                    elbow,
                    grabber,
                    claw,
                    tilt,
                    ElementType.CONE,
                    PlacementType.HIGH);
        } else if (name.contains("ScoreCubeHigh")) {
            return ArmAutoFactories.AlignmentAndPlacement.create(
                    elevator,
                    elbow,
                    grabber,
                    claw,
                    tilt,
                    ElementType.CUBE,
                    PlacementType.HIGH);
        } else if (name.contains("ScoreConeMiddle")) {
            return ArmAutoFactories.AlignmentAndPlacement.create(
                    elevator,
                    elbow,
                    grabber,
                    claw,
                    tilt,
                    ElementType.CONE,
                    PlacementType.MIDDLE);
        } else if (name.contains("ScoreCubeMiddle")) {
            return ArmAutoFactories.AlignmentAndPlacement.create(
                    elevator,
                    elbow,
                    grabber,
                    claw,
                    tilt,
                    ElementType.CUBE,
                    PlacementType.MIDDLE);
        } else if (name.contains("ScoreConeLow")) {
            return ArmAutoFactories.AlignmentAndPlacement.create(
                    elevator,
                    elbow,
                    grabber,
                    claw,
                    tilt,
                    ElementType.CONE,
                    PlacementType.LOW);
        } else if (name.contains("ScoreCubeLow")) {
            return ArmAutoFactories.AlignmentAndPlacement.create(
                    elevator,
                    elbow,
                    grabber,
                    claw,
                    tilt,
                    ElementType.CUBE,
                    PlacementType.LOW);
        } else if (name.contains("PickupCone")) {
            return ArmCommandFactories.Pickup.create(
                    elevator,
                    elbow,
                    grabber,
                    claw,
                    tilt,
                    Constants.Auto.Arm.Pickup.Floor.elevatorPositionMeters,
                    Constants.Auto.Arm.Pickup.Floor.Cone.elbowPositionDegrees,
                    Constants.Auto.Arm.Pickup.grabberSpeedRPM,
                    Constants.Auto.Arm.Pickup.Floor.elevatorTiltState,
                    Claw.State.CONE);
        } else if (name.contains("PickupCube")) {
            return ArmCommandFactories.Pickup.create(
                    elevator,
                    elbow,
                    grabber,
                    claw,
                    tilt,
                    Constants.Auto.Arm.Pickup.Floor.elevatorPositionMeters,
                    Constants.Auto.Arm.Pickup.Floor.Cube.elbowPositionDegrees,
                    Constants.Auto.Arm.Pickup.grabberSpeedRPM,
                    Constants.Auto.Arm.Pickup.Floor.elevatorTiltState,
                    Claw.State.CUBE);
        } else if(name.contains("ResetTelemetry")){
            return swerveDrive.setTelemetryFromCameraCommand();
        }
        else if (name.contains("Carry")) {
            return ArmCommandFactories.Carry.create(
                    elevator,
                    elbow,
                    grabber,
                    tilt,
                    Constants.Auto.Arm.Carry.elevatorPositionMeters,
                    Constants.Auto.Arm.Carry.elbowPositionDegrees,
                    Constants.Auto.Arm.Carry.elevatorTiltState);
        } else if (name.contains("ElevatorUp")) {
            return elevator.createControlCommand(0.5);
        } else if (name.contains("DanceParty")) {
            return ledLighting.getDanceParty();
        } else {
            return Commands.none();
        }
    }

}
