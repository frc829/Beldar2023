// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Telemetry;

/** Add your docs here. */
public abstract class Chassis {

    private static PIDController forwardController = new PIDController(5, 0, 0);
    private static PIDController strafeController = new PIDController(5, 0, 0);
    private static PIDController rotationController = new PIDController(5, 0, 0);

    public static CommandBase createManualDriveCommand(
            SwerveDrive swerveDrive,
            Telemetry telemetry) {
        CommandBase manualDriveCommand = new CommandBase() {
            @Override
            public void execute() {
                Rotation2d robotRotation = telemetry.getSwerveDrivePosition().getRotation();
                ChassisSpeeds chassisSpeeds = swerveDrive.getRobotCentricSpeeds(robotRotation);
                swerveDrive.setSwerveDriveChassisSpeed(chassisSpeeds);
            }
        };

        manualDriveCommand.addRequirements(swerveDrive, telemetry);

        return manualDriveCommand;

    }

    public static CommandBase getBalanceTestingCommand2(
            SwerveDrive swerveDrive,
            Telemetry telemetry) {

        CommandBase balance = new CommandBase() {

            @Override
            public void initialize() {
                SmartDashboard.putString("Swerve Drive Current Command", "Balancing");
            }

            @Override
            public void execute() {
                Rotation2d pitchAngle = telemetry.getPitch();
                Rotation2d rollAngle = telemetry.getRoll();
                Rotation2d pitchDistanceFrom0 = pitchAngle.minus(new Rotation2d());
                Rotation2d rollDistanceFrom0 = rollAngle.minus(new Rotation2d());
                double pitchDistanceFrom0Radians = pitchDistanceFrom0.getRadians();
                double rollDistatnceFrom0Radians = rollDistanceFrom0.getRadians();

                double vxMetersPerSecond = -Constants.Robot.Drive.Modules.maxModuleSpeedMPS
                        * Math.sin(pitchDistanceFrom0Radians) / 2.5;

                double vyMetersPerSecond = Constants.Robot.Drive.Modules.maxModuleSpeedMPS
                        * Math.sin(rollDistatnceFrom0Radians) / 2.5;

                vxMetersPerSecond = MathUtil.applyDeadband(vxMetersPerSecond, 0.10);
                vyMetersPerSecond = MathUtil.applyDeadband(vyMetersPerSecond, 0.10);
                swerveDrive.setSwerveDriveChassisSpeed(new ChassisSpeeds(vxMetersPerSecond, 0, 0));
            }

            @Override
            public void end(boolean interrupted) {
                swerveDrive.stopDrive();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };

        balance.addRequirements(swerveDrive, telemetry);
        return balance;
    }

    public static CommandBase getBalanceTestingCommand(
            SwerveDrive swerveDrive,
            Telemetry telemetry) {

        CommandBase balance = new CommandBase() {

            @Override
            public void initialize() {
                SmartDashboard.putString("Swerve Drive Current Command", "Balancing");
            }

            @Override
            public void execute() {
                Rotation2d pitchAngle = telemetry.getPitch();
                Rotation2d rollAngle = telemetry.getRoll();
                Rotation2d pitchDistanceFrom0 = pitchAngle.minus(new Rotation2d());
                Rotation2d rollDistanceFrom0 = rollAngle.minus(new Rotation2d());
                double pitchDistanceFrom0Radians = pitchDistanceFrom0.getRadians();
                double rollDistatnceFrom0Radians = rollDistanceFrom0.getRadians();

                double vxMetersPerSecond = -Constants.Robot.Drive.Modules.maxModuleSpeedMPS
                        * Math.sin(pitchDistanceFrom0Radians) / 2.5;

                double vyMetersPerSecond = Constants.Robot.Drive.Modules.maxModuleSpeedMPS
                        * Math.sin(rollDistatnceFrom0Radians) / 2.5;

                vxMetersPerSecond = MathUtil.applyDeadband(vxMetersPerSecond, 0.10);
                vyMetersPerSecond = MathUtil.applyDeadband(vyMetersPerSecond, 0.10);
                swerveDrive.setSwerveDriveChassisSpeed(new ChassisSpeeds(vxMetersPerSecond, 0, 0));
            }

            @Override
            public void end(boolean interrupted) {
                swerveDrive.stopDrive();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };

        balance.addRequirements(swerveDrive, telemetry);
        return balance;
    }

    public static CommandBase createSlidingPortalCommand(
            SwerveDrive swerveDrive,
            Telemetry telemetry,
            Pose2d bluePortal,
            Pose2d redPortal) {

        rotationController.enableContinuousInput(0, 1);

        CommandBase slidingPortalCommand = new CommandBase() {

            @Override
            public void initialize() {
                Pose2d goalPosition = bluePortal;
                if (DriverStation.getAlliance() == Alliance.Red) {
                    goalPosition = redPortal;
                    goalPosition = new Pose2d(goalPosition.getX(), 8.02 - goalPosition.getY(),
                            goalPosition.getRotation());
                }
                forwardController.setSetpoint(goalPosition.getX());
                strafeController.setSetpoint(goalPosition.getY());
                rotationController.setSetpoint(goalPosition.getRotation().getRotations());
            }

            @Override
            public void execute() {
                Pose2d currentPosition = telemetry.getSwerveDrivePosition();
                double vxMetersPerSecond = forwardController.calculate(currentPosition.getX());
                double vyMetersPerSecond = strafeController.calculate(currentPosition.getY());
                double rotationSpeedRPS = rotationController.calculate(currentPosition.getRotation().getRotations());
                double omegaRadiansPerSecond = rotationSpeedRPS * Math.PI * 2;
                ChassisSpeeds fcChassisSpeed = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond,
                        omegaRadiansPerSecond);
                ChassisSpeeds rcChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fcChassisSpeed,
                        currentPosition.getRotation());
                swerveDrive.setSwerveDriveChassisSpeed(rcChassisSpeeds);
            }

            @Override
            public void end(boolean interrupted) {
                swerveDrive.stopDrive();
            }

            @Override
            public boolean isFinished() {
                return forwardController.atSetpoint() && strafeController.atSetpoint()
                        && rotationController.atSetpoint();
            }

        };

        slidingPortalCommand.addRequirements(swerveDrive, telemetry);
        return slidingPortalCommand;

    }

    public static CommandBase createDropPortalCommand(
            SwerveDrive swerveDrive,
            Telemetry telemetry,
            Pose2d bluePortal) {

        rotationController.enableContinuousInput(0, 1);

        CommandBase dropPortalCommand = new CommandBase() {

            @Override
            public void initialize() {
                Pose2d goalPosition = bluePortal;
                if (DriverStation.getAlliance() == Alliance.Red) {
                    goalPosition = new Pose2d(goalPosition.getX(), 8.02 - goalPosition.getY(),
                            goalPosition.getRotation().unaryMinus());
                }
                forwardController.setSetpoint(goalPosition.getX());
                strafeController.setSetpoint(goalPosition.getY());
                rotationController.setSetpoint(goalPosition.getRotation().getRotations());
            }

            @Override
            public void execute() {
                Pose2d currentPosition = telemetry.getSwerveDrivePosition();
                double vxMetersPerSecond = forwardController.calculate(currentPosition.getX());
                double vyMetersPerSecond = strafeController.calculate(currentPosition.getY());
                double rotationSpeedRPS = rotationController.calculate(currentPosition.getRotation().getRotations());
                double omegaRadiansPerSecond = rotationSpeedRPS * Math.PI * 2;
                ChassisSpeeds fcChassisSpeed = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond,
                        omegaRadiansPerSecond);
                ChassisSpeeds rcChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fcChassisSpeed,
                        currentPosition.getRotation());
                swerveDrive.setSwerveDriveChassisSpeed(rcChassisSpeeds);
            }

            @Override
            public void end(boolean interrupted) {
                swerveDrive.stopDrive();
            }

            @Override
            public boolean isFinished() {
                return forwardController.atSetpoint() && strafeController.atSetpoint()
                        && rotationController.atSetpoint();
            }

        };

        dropPortalCommand.addRequirements(swerveDrive, telemetry);
        return dropPortalCommand;

    }

    public static CommandBase createNearestPointCommand(
            SwerveDrive swerveDrive,
            Telemetry telemetry,
            List<Pose2d> scoringPositions) {

        rotationController.enableContinuousInput(0, 1);

        CommandBase nearestPointCommand = new CommandBase() {

            @Override
            public void initialize() {

                Pose2d goalPosition = telemetry.getClosestPosition(scoringPositions);

                forwardController.setSetpoint(goalPosition.getX());
                strafeController.setSetpoint(goalPosition.getY());
                rotationController.setSetpoint(goalPosition.getRotation().getRotations());
            }

            @Override
            public void execute() {
                Pose2d currentPosition = telemetry.getSwerveDrivePosition();
                double vxMetersPerSecond = forwardController.calculate(currentPosition.getX());
                double vyMetersPerSecond = strafeController.calculate(currentPosition.getY());
                double rotationSpeedRPS = rotationController.calculate(currentPosition.getRotation().getRotations());
                double omegaRadiansPerSecond = rotationSpeedRPS * Math.PI * 2;
                ChassisSpeeds fcChassisSpeed = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond,
                        omegaRadiansPerSecond);
                ChassisSpeeds rcChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fcChassisSpeed,
                        currentPosition.getRotation());
                swerveDrive.setSwerveDriveChassisSpeed(rcChassisSpeeds);
            }

            @Override
            public void end(boolean interrupted) {
                swerveDrive.stopDrive();
            }

            @Override
            public boolean isFinished() {
                return forwardController.atSetpoint() && strafeController.atSetpoint()
                        && rotationController.atSetpoint();
            }

        };

        nearestPointCommand.addRequirements(swerveDrive, telemetry);
        return nearestPointCommand;

    }

}
