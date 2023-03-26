// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.AutoRoutines.Element1.testing1;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTilt;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Telemetry;

/** Add your docs here. */
public class Balancing {

    public static CommandBase createCube(
            SwerveDrive swerveDrive,
            Telemetry telemetry,
            Elevator elevator,
            Elbow elbow,
            Grabber grabber,
            ElevatorTilt tilt,
            Claw claw) {

        CommandBase resetPose = Commands.runOnce(
                () -> {

                    telemetry.resetSwerveDrivePosition(new Pose2d(
                            1.81,
                            2.75,
                            Rotation2d.fromDegrees(180)));
                },
                telemetry);

        CommandBase clawCube = claw.createSetStateCommand(Claw.State.CUBE);
        CommandBase alignment = Arm.createAlignHigh(elevator, elbow, tilt, claw);
        CommandBase placement = Arm.createHighPlacement(elevator, elbow, tilt, claw, grabber);
        CommandBase reset = Arm.createResetHigh(elevator, elbow, tilt, grabber);
        CommandBase scoreHighCube = Commands.sequence(clawCube, alignment, placement, reset);

        CommandBase driveBackwards = Commands.run(
                () -> {
                    swerveDrive.setSwerveDriveChassisSpeed(new ChassisSpeeds(-2, 0, 0));
                },
                swerveDrive);

        CommandBase driveBackwardsDeadline = Commands.waitSeconds(1);

        CommandBase phase1 = Commands.deadline(driveBackwardsDeadline, driveBackwards);

        CommandBase balance = Commands.run(
                () -> {
                    SmartDashboard.putString("Balance State", "Yes");

                    Rotation2d pitchAngle = telemetry.getPitch();
                    Rotation2d pitchDistanceFrom0 = pitchAngle.minus(new Rotation2d());
                    double vxMetersPerSecond = -Constants.Robot.Drive.Modules.maxModuleSpeedMPS
                            * Math.sin(pitchDistanceFrom0.getRadians()) / 2.4;
                    SmartDashboard.putNumber("VVVVVXXXXX", vxMetersPerSecond);
                    SmartDashboard.putNumber("AAAANGGGLLE", pitchDistanceFrom0.getDegrees());
                    SmartDashboard.putNumber("SIGN OF ANGLE", Math.signum(pitchDistanceFrom0.getDegrees()));
                    swerveDrive.setSwerveDriveChassisSpeed(new ChassisSpeeds(vxMetersPerSecond, 0, 0));
                },
                swerveDrive);

        return Commands.sequence(resetPose, scoreHighCube, phase1, balance);

    }

}
