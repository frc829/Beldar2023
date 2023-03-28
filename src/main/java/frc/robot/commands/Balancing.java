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
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTilt;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.LEDLighting;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Telemetry;

/** Add your docs here. */
public class Balancing {

        public static CommandBase WeComeFromFrance(
                        SwerveDrive swerveDrive,
                        Telemetry telemetry,
                        Elevator elevator,
                        Elbow elbow,
                        Grabber grabber,
                        ElevatorTilt tilt,
                        Claw claw,
                        LEDLighting ledLighting,
                        double getOverRampTimeDeadLine,
                        double settlingWait,
                        double getOnRampTimeDeadLine,
                        double driveUpHillSpeed,
                        double driveUpHillTimeDeadline,
                        double balanceSpeedScaleFactor) {

                CommandBase limeLightOff = telemetry.turnOffTrackingCamera();

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

                CommandBase smallWait = Commands.waitSeconds(settlingWait);

                CommandBase driveBackwardsDeadline = Commands.waitSeconds(getOverRampTimeDeadLine);

                CommandBase phase1 = Commands.deadline(driveBackwardsDeadline, driveBackwards);

                CommandBase driveForwardsCommand = Commands.run(
                                () -> {
                                        swerveDrive.setSwerveDriveChassisSpeed(new ChassisSpeeds(2, 0, 0));
                                },
                                swerveDrive);

                CommandBase driveForwardsDeadline = Commands.waitSeconds(getOverRampTimeDeadLine);

                CommandBase phase2 = Commands.deadline(driveForwardsDeadline, driveForwardsCommand);

                CommandBase driveUpHill = Commands.run(
                                () -> {
                                        double vxMetersPerSecond = driveUpHillSpeed;
                                        swerveDrive.setSwerveDriveChassisSpeed(
                                                        new ChassisSpeeds(vxMetersPerSecond, 0, 0));
                                },
                                swerveDrive);

                CommandBase driveUpHillDeadeLine = Commands.waitSeconds(driveUpHillTimeDeadline);

                CommandBase phase3 = Commands.deadline(driveUpHillDeadeLine, driveUpHill);

                CommandBase balance = Commands.run(
                                () -> {
                                        SmartDashboard.putString("01:Balance State", "Yes");

                                        Rotation2d pitchAngle = telemetry.getPitch();
                                        Rotation2d pitchDistanceFrom0 = pitchAngle.minus(new Rotation2d());
                                        double vxMetersPerSecond = balanceSpeedScaleFactor
                                                        * Math.sin(pitchDistanceFrom0.getRadians());
                                        SmartDashboard.putNumber("02:VX", vxMetersPerSecond);
                                        SmartDashboard.putNumber("03:ANGLE", pitchDistanceFrom0.getDegrees());
                                        SmartDashboard.putNumber("04:SIGN OF ANGLE",
                                                        Math.signum(pitchDistanceFrom0.getDegrees()));
                                        swerveDrive.setSwerveDriveChassisSpeed(
                                                        new ChassisSpeeds(vxMetersPerSecond, 0, 0));
                                },
                                swerveDrive);

                CommandBase phase1Lighting = ledLighting.getDriveBackwardLighting();
                CommandBase phase2Lighting = ledLighting.getDriveForwardLighting();
                CommandBase phase3Lighting = ledLighting.getDriveUpHillLighting();
                CommandBase balanceLighting = ledLighting.getDanceParty();

                return Commands.sequence(resetPose, scoreHighCube, limeLightOff, phase1Lighting, phase1, smallWait,
                                phase2Lighting, phase2,
                                phase3Lighting, phase3,
                                balanceLighting, balance);
        }

        public static CommandBase RobbiesBalanceImproved(
                        SwerveDrive swerveDrive,
                        Telemetry telemetry,
                        Elevator elevator,
                        Elbow elbow,
                        Grabber grabber,
                        ElevatorTilt tilt,
                        Claw claw,
                        LEDLighting ledLighting,
                        double getOnRampTimeDeadLine,
                        double driveUpHillSpeed,
                        double driveUpHillTimeDeadline,
                        double balanceSpeedScaleFactor) {

                CommandBase limeLightOff = telemetry.turnOffTrackingCamera();

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

                CommandBase driveBackwardsDeadline = Commands.waitSeconds(getOnRampTimeDeadLine);

                CommandBase phase1 = Commands.deadline(driveBackwardsDeadline, driveBackwards);

                CommandBase driveUpHill = Commands.run(
                                () -> {
                                        double vxMetersPerSecond = driveUpHillSpeed;
                                        swerveDrive.setSwerveDriveChassisSpeed(
                                                        new ChassisSpeeds(vxMetersPerSecond, 0, 0));
                                },
                                swerveDrive);

                CommandBase driveUpHillDeadeLine = Commands.waitSeconds(driveUpHillTimeDeadline);

                CommandBase phase2 = Commands.deadline(driveUpHillDeadeLine, driveUpHill);

                CommandBase balance = Commands.run(
                                () -> {
                                        SmartDashboard.putString("01:Balance State", "Yes");

                                        Rotation2d pitchAngle = telemetry.getPitch();
                                        Rotation2d pitchDistanceFrom0 = pitchAngle.minus(new Rotation2d());
                                        double vxMetersPerSecond = balanceSpeedScaleFactor
                                                        * Math.sin(pitchDistanceFrom0.getRadians());
                                        SmartDashboard.putNumber("02:VX", vxMetersPerSecond);
                                        SmartDashboard.putNumber("03:ANGLE", pitchDistanceFrom0.getDegrees());
                                        SmartDashboard.putNumber("04:SIGN OF ANGLE",
                                                        Math.signum(pitchDistanceFrom0.getDegrees()));
                                        swerveDrive.setSwerveDriveChassisSpeed(
                                                        new ChassisSpeeds(vxMetersPerSecond, 0, 0));
                                },
                                swerveDrive);

                CommandBase phase1Lighting = ledLighting.getDriveBackwardLighting();
                CommandBase phase2Lighting = ledLighting.getDriveUpHillLighting();
                CommandBase balanceLighting = ledLighting.getDanceParty();

                return Commands.sequence(resetPose, scoreHighCube, limeLightOff, phase1Lighting, phase1, phase2Lighting,
                                phase2,
                                balanceLighting, balance);
        }

        public static CommandBase RobbiesBalance(
                        SwerveDrive swerveDrive,
                        Telemetry telemetry,
                        Elevator elevator,
                        Elbow elbow,
                        Grabber grabber,
                        ElevatorTilt tilt,
                        Claw claw) {

                CommandBase limeLightOff = telemetry.turnOffTrackingCamera();

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

                CommandBase driveBackwardsDeadline = Commands.waitSeconds(1.70);

                CommandBase phase1 = Commands.deadline(driveBackwardsDeadline, driveBackwards);

                CommandBase balance = Commands.run(
                                () -> {
                                        SmartDashboard.putString("Balance State", "Yes");

                                        Rotation2d pitchAngle = telemetry.getPitch();
                                        Rotation2d pitchDistanceFrom0 = pitchAngle.minus(new Rotation2d());
                                        double vxMetersPerSecond = -Constants.Robot.Drive.Modules.maxModuleSpeedMPS
                                                        * Math.sin(pitchDistanceFrom0.getRadians()) / 2.35;
                                        SmartDashboard.putNumber("VVVVVXXXXX", vxMetersPerSecond);
                                        SmartDashboard.putNumber("AAAANGGGLLE", pitchDistanceFrom0.getDegrees());
                                        SmartDashboard.putNumber("SIGN OF ANGLE",
                                                        Math.signum(pitchDistanceFrom0.getDegrees()));
                                        if (vxMetersPerSecond >= 0) {
                                                vxMetersPerSecond = 0;
                                        }
                                        swerveDrive.setSwerveDriveChassisSpeed(
                                                        new ChassisSpeeds(vxMetersPerSecond, 0, 0));
                                },
                                swerveDrive);

                return Commands.sequence(resetPose, scoreHighCube, limeLightOff, phase1, balance);

        }

        public static CommandBase ConquerTheBluntSkulls(
                        SwerveDrive swerveDrive,
                        Telemetry telemetry,
                        Elevator elevator,
                        Elbow elbow,
                        Grabber grabber,
                        ElevatorTilt tilt,
                        LEDLighting ledLighting,
                        Claw claw) {

                Pose2d startPosition = new Pose2d(1.89, 4.43, Rotation2d.fromDegrees(180));
                Pose2d firstPosition = new Pose2d(4.05, 4.78, Rotation2d.fromDegrees(180));
                Pose2d secondPosition = new Pose2d(6.04, 4.64, Rotation2d.fromDegrees(0));
                Pose2d firstCubePosition = new Pose2d(7.08, 4.58, Rotation2d.fromDegrees(0));
                Pose2d firstCubePosition2 = new Pose2d(7.08, 4.58, Rotation2d.fromDegrees(180));
                Pose2d firstCubePosition3 = new Pose2d(7.08, 4.58, Rotation2d.fromDegrees(270));

                CommandBase resetPose = Commands.runOnce(
                                () -> {

                                        telemetry.resetSwerveDrivePosition(startPosition);
                                },
                                telemetry);

                CommandBase clawCube = claw.createSetStateCommand(Claw.State.CUBE);
                CommandBase alignment = Arm.createAlignHigh(elevator, elbow, tilt, claw);
                CommandBase placement = Arm.createHighPlacement(elevator, elbow, tilt, claw, grabber);
                CommandBase reset = Arm.createResetHigh(elevator, elbow, tilt, grabber);
                CommandBase scoreHighCube = Commands.sequence(clawCube, alignment, placement, reset);

                CommandBase goToFirstPointCommand = Chassis.createDropPortalCommand(
                                swerveDrive, telemetry, firstPosition);

                CommandBase goToSecondPointCommand = Chassis.createDropPortalCommand(
                                swerveDrive, telemetry, secondPosition);

                CommandBase gotToFirstCube = Chassis.createDropPortalCommand(
                                swerveDrive, telemetry, firstCubePosition);

                CommandBase createCubePickup = Arm.createFloorPickup(
                                elevator, elbow, tilt, grabber, claw,
                                ledLighting, Claw.State.CUBE);

                CommandBase pickupCube = Commands.race(createCubePickup, gotToFirstCube);

                CommandBase carry1 = Arm.createCarry(elevator, elbow, tilt, claw);

                CommandBase goBackToSecondPointCommand = Chassis.createDropPortalCommand(
                                swerveDrive, telemetry, secondPosition);

                CommandBase goToBackFirstPointCommand = Chassis.createDropPortalCommand(
                                swerveDrive, telemetry, firstPosition);

                CommandBase clawCube2 = claw.createSetStateCommand(Claw.State.CUBE);
                CommandBase alignment2 = Arm.createAlignMiddle(elevator, elbow, tilt, claw);
                CommandBase placement2 = Arm.createMiddlePlacement(elevator, elbow, tilt, claw, grabber);
                CommandBase reset2 = Arm.createResetMiddle(elevator, elbow, tilt, grabber);
                CommandBase scoreHighCube2 = Commands.sequence(clawCube2, alignment2, placement2, reset2);

                CommandBase goToBackStartPointCommand = Chassis.createDropPortalCommand(
                                swerveDrive, telemetry, startPosition);

                CommandBase goBackToCubePointCommand = Chassis.createDropPortalCommand(
                                swerveDrive,
                                telemetry,
                                firstCubePosition2);

                CommandBase rotate90Command = Chassis.createDropPortalCommand(
                                swerveDrive,
                                telemetry,
                                firstCubePosition3);

                return Commands.sequence(resetPose, scoreHighCube, goToFirstPointCommand, goToSecondPointCommand,
                                pickupCube, carry1, goBackToSecondPointCommand, goToBackFirstPointCommand,
                                goToBackStartPointCommand,
                                scoreHighCube2, goBackToCubePointCommand, rotate90Command);

        }

}
