// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class OnThefly extends CommandBase {
  private Command drivingCommand;
  private Pose2d goalPose;
  private final Field2d theField;
  private final SwerveDrive swerveDrive;

  public OnThefly(Pose2d goalPose, SwerveDrive swerveDrive, Field2d theField) {

    this.goalPose = goalPose;
    this.swerveDrive = swerveDrive;
    this.theField = theField;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d initialPose = swerveDrive.getSwerveDrivePosition();
    if (DriverStation.getAlliance() == Alliance.Red) {
      initialPose = new Pose2d(
          initialPose.getX(),
          8.02 - initialPose.getY(),
          new Rotation2d().minus(initialPose.getRotation()));
    }

    PathPoint initialPoint = PathPoint.fromCurrentHolonomicState(
        initialPose,
        swerveDrive.getSwerveDriveChassisSpeed());

    PathPoint goalPoint = PathPoint.fromCurrentHolonomicState(goalPose, new ChassisSpeeds());

    PathPlannerTrajectory trajectory = PathPlanner.generatePath(
        new PathConstraints(3, 3),
        initialPoint,
        goalPoint);

    trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());

    this.drivingCommand = new PPSwerveControllerCommand(
        trajectory,
        swerveDrive::getSwerveDrivePosition,
        new PIDController(4, 0, 0),
        new PIDController(4, 0, 0),
        new PIDController(5, 0, 0),
        swerveDrive::setSwerveDriveChassisSpeed,
        true);

    List<Pose2d> poses = new ArrayList<Pose2d>();
    for (var state : trajectory.getStates()) {
      poses.add(state.poseMeters);
    }
    theField.getObject("Trajectory").setPoses(poses);

    drivingCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivingCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivingCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivingCommand.isFinished();
  }
}
