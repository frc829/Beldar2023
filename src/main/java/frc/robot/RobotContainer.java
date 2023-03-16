// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Arm;
import frc.robot.commands.PathPlannerToAuto;
import frc.robot.commands.Arm.PlacementAndReset;
import frc.robot.framework.kinematics.KinematicsFactory;
import frc.robot.framework.telemetry.FieldMap;
import frc.robot.framework.vision.DumbOldCamera;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTilt;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.LEDLighting;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Telemetry;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        private final CommandXboxController driveController;
        private final CommandXboxController operatorController;
        private final FieldMap fieldMap;

        private final SwerveDrive swerveDrive;
        private final Claw claw;
        private final Grabber grabber;
        private final Elevator elevator;
        private final Elbow elbow;
        private final ElevatorTilt tilt;
        private final LEDLighting ledLighting;

        private final HashMap<String, Command> autoCommands;
        private final HashMap<String, List<PathPlannerTrajectory>> pathPlannerTrajectories;
        private final SendableChooser<String> autoChooser = new SendableChooser<>();
        private final Telemetry telemetry;

        public enum AutoBalanceDirection {
                Forward,
                Backward
        }

        public RobotContainer() {

                DumbOldCamera.start();
                SwerveDriveKinematics swerveDriveKinematics = KinematicsFactory.createSwerveKinematics(
                                Constants.Robot.Drive.Modules.FrontLeft.location,
                                Constants.Robot.Drive.Modules.FrontRight.location,
                                Constants.Robot.Drive.Modules.RearLeft.location,
                                Constants.Robot.Drive.Modules.RearRight.location);

                this.driveController = new CommandXboxController(
                                OperatorConstants.DriverController.kDriverControllerPort);
                this.operatorController = new CommandXboxController(
                                OperatorConstants.DriverController.kOperatorControllerPort);
                this.fieldMap = new FieldMap();
                this.swerveDrive = new SwerveDrive(driveController, fieldMap, swerveDriveKinematics);
                this.swerveDrive.setManualDefaultCommand();

                Pose2d initialPosition = new Pose2d(3, 3, new Rotation2d());
                this.telemetry = new Telemetry(initialPosition, fieldMap, swerveDriveKinematics,
                                swerveDrive.getSwerveModules());

                Mechanism2d mech = new Mechanism2d(3, 3);
                MechanismRoot2d root = mech.getRoot("Arm", 2, 1);
                MechanismLigament2d elevatorMech2d = root.append(
                                new MechanismLigament2d(
                                                "elevator",
                                                1,
                                                90,
                                                0,
                                                new Color8Bit(Color.kBlack)));

                elevatorMech2d.setColor(new Color8Bit(Color.kBlack));

                MechanismLigament2d fakeElevatorMech2d = root.append(
                                new MechanismLigament2d(
                                                "fakeElevator",
                                                1,
                                                90,
                                                1,
                                                new Color8Bit(Color.kGold)));

                MechanismLigament2d elbowMech2d = elevatorMech2d.append(
                                new MechanismLigament2d(
                                                "elbow",
                                                .50,
                                                10,
                                                6,
                                                new Color8Bit(Color.kPurple)));

                SmartDashboard.putData("Arm2d", mech);

                this.tilt = new ElevatorTilt(fakeElevatorMech2d, elevatorMech2d);
                CommandBase defaultElevatorTiltCommand = tilt.createIdleCommand();
                this.tilt.setDefaultCommand(defaultElevatorTiltCommand);

                this.claw = new Claw();
                claw.setDefaultCommand(claw.createIdleCommand());

                this.grabber = new Grabber(operatorController);
                this.grabber.setDefaultCommand(this.grabber.createStopCommand());
                this.grabber.setManualControlTrigger();

                this.elevator = new Elevator(operatorController, elevatorMech2d);
                this.elevator.setDefaultCommand(this.elevator.createHoldCommand());
                this.elevator.setManualControlTrigger();

                this.elbow = new Elbow(operatorController, elbowMech2d);
                this.elbow.setDefaultCommand(this.elbow.createHoldCommand());
                this.elbow.setManualControlTrigger();

                this.ledLighting = new LEDLighting();

                pathPlannerTrajectories = new HashMap<>();
                autoCommands = new HashMap<>();

                // Configure the trigger bindings
                configureDriverBindings();
                configureOperatorBindings();
                configureAutoCommands();
        }

        private void configureDriverBindings() {

                CommandBase zeroModulesCommand = swerveDrive.getZeroModuleCommand();
                CommandBase setTelemetryFromCameraCommand = telemetry.setTelemetryFromCameraCommand();

                CommandBase dropPortalAlign = swerveDrive
                                .createDropPortalCommand(Constants.Auto.Drive.PortalPositions.dropPortal);

                CommandBase leftPortalAlign = swerveDrive.createSlidingPortalCommand(
                                Constants.Auto.Drive.PortalPositions.leftPortal,
                                Constants.Auto.Drive.PortalPositions.rightPortal);

                CommandBase rightPortalAlign = swerveDrive.createSlidingPortalCommand(
                                Constants.Auto.Drive.PortalPositions.rightPortal,
                                Constants.Auto.Drive.PortalPositions.leftPortal);

                CommandBase nearestScoreAlign = swerveDrive.createNearestPointCommand(
                                Constants.Auto.Drive.ScoringPositions.positionsList);

                CommandBase highConePoofCommand = PlacementAndReset.createHighConePoof(
                                elevator, elbow, tilt, claw, grabber);

                CommandBase highPlacementAndResetCommand = PlacementAndReset.createHighPlacementAndReset(
                                elevator, elbow, tilt, claw, grabber);

                CommandBase middlePlacementAndResetCommand = PlacementAndReset.createMiddlePlacementAndReset(
                                elevator, elbow, tilt, claw, grabber);

                CommandBase highAlignment = Arm.Alignment.createHigh(elevator, elbow, tilt, claw);
                CommandBase middleAlignment = Arm.Alignment.createMiddle(elevator, elbow, tilt, claw);
                CommandBase lowAlignPlacmentAndResetCommand = PlacementAndReset
                                .createLowAlignPlacementAndReset(elevator, elbow, tilt, claw, grabber);

                driveController.back().onTrue(setTelemetryFromCameraCommand);
                driveController.start().whileTrue(zeroModulesCommand);
                driveController.b().onTrue(highConePoofCommand);
                driveController.x().onTrue(middleAlignment);
                driveController.y().onTrue(highAlignment);
                driveController.a().onTrue(lowAlignPlacmentAndResetCommand);
                driveController.leftBumper().onTrue(middlePlacementAndResetCommand);
                driveController.rightBumper().onTrue(highPlacementAndResetCommand);
                driveController.povLeft().whileTrue(leftPortalAlign);
                driveController.povUp().whileTrue(nearestScoreAlign);
                driveController.povRight().whileTrue(rightPortalAlign);
                driveController.povDown().whileTrue(dropPortalAlign);

        }

        private void configureOperatorBindings() {

                CommandBase elementCarry = Arm.Carry.create(
                                elevator, elbow, grabber, tilt, claw);

                CommandBase conePickupFloor = Arm.Pickup.createFloor(
                                elevator, elbow, grabber, claw, tilt, Claw.State.CONE, ledLighting);

                CommandBase cubePickupFloor = Arm.Pickup.createFloor(
                                elevator, elbow, grabber, claw, tilt, Claw.State.CUBE, ledLighting);

                CommandBase conePickupSliding = Arm.Pickup.createSliding(
                                elevator, elbow, grabber, claw, tilt, Claw.State.CONE, ledLighting);

                CommandBase cubePickupSliding = Arm.Pickup.createSliding(
                                elevator, elbow, grabber, claw, tilt, Claw.State.CUBE, ledLighting);

                CommandBase conePickupDrop = Arm.Pickup.createDrop(
                                elevator, elbow, grabber, claw, tilt, Claw.State.CONE, ledLighting);

                CommandBase cubePickupDrop = Arm.Pickup.createDrop(
                                elevator, elbow, grabber, claw, tilt, Claw.State.CUBE, ledLighting);

                CommandBase grabberToggleCommand = Arm.ClawControl.createToggle(claw, ledLighting);

                CommandBase noLegsCommand = tilt.createControlCommand(ElevatorTilt.State.NONE);
                CommandBase shortSaber = tilt.createControlCommand(ElevatorTilt.State.TWO);
                CommandBase tatooine = tilt.createControlCommand(ElevatorTilt.State.SIX);
                CommandBase duelOfTheFates = tilt.createControlCommand(ElevatorTilt.State.EIGHT);
                CommandBase danceParty = ledLighting.getDanceParty2();

                operatorController.rightBumper().whileTrue(cubePickupFloor);
                operatorController.rightBumper().onFalse(elementCarry);
                operatorController.leftBumper().whileTrue(conePickupFloor);
                operatorController.leftBumper().onFalse(elementCarry);
                operatorController.b().whileTrue(cubePickupDrop);
                operatorController.b().onFalse(elementCarry);
                operatorController.x().whileTrue(conePickupDrop);
                operatorController.x().onFalse(elementCarry);
                operatorController.a().whileTrue(cubePickupSliding);
                operatorController.a().onFalse(elementCarry);
                operatorController.y().whileTrue(conePickupSliding);
                operatorController.y().onFalse(elementCarry);
                operatorController.back().onTrue(grabberToggleCommand);
                operatorController.povDown().onTrue(noLegsCommand);
                operatorController.povLeft().onTrue(shortSaber);
                operatorController.povRight().onTrue(tatooine);
                operatorController.povUp().onTrue(duelOfTheFates);
                operatorController.start().whileTrue(danceParty);
        }

        private void configureAutoCommands() {
                addAutoCommand(
                                Constants.AutoRoutines.Element2.position2.pathName,
                                Constants.AutoRoutines.Element2.position2.firstPathConstraint,
                                Constants.AutoRoutines.Element2.position2.remainingPathConstraints,
                                Constants.AutoRoutines.Element2.position2.translationConstants,
                                Constants.AutoRoutines.Element2.position2.rotationConstants,
                                Commands.none());

                addAutoCommand(
                                Constants.AutoRoutines.Element1.position4.pathName,
                                Constants.AutoRoutines.Element1.position4.firstPathConstraint,
                                Constants.AutoRoutines.Element1.position4.remainingPathConstraints,
                                Constants.AutoRoutines.Element1.position4.translationConstants,
                                Constants.AutoRoutines.Element1.position4.rotationConstants,
                                AutoBalanceDirection.Backward);

                addAutoCommand(
                                Constants.AutoRoutines.Element1.position5.pathName,
                                Constants.AutoRoutines.Element1.position5.firstPathConstraint,
                                Constants.AutoRoutines.Element1.position5.remainingPathConstraints,
                                Constants.AutoRoutines.Element1.position5.translationConstants,
                                Constants.AutoRoutines.Element1.position5.rotationConstants,
                                AutoBalanceDirection.Backward);

                addAutoCommand(
                                Constants.AutoRoutines.Element1.position5Balance.pathName,
                                Constants.AutoRoutines.Element1.position5Balance.firstPathConstraint,
                                Constants.AutoRoutines.Element1.position5Balance.remainingPathConstraints,
                                Constants.AutoRoutines.Element1.position5Balance.translationConstants,
                                Constants.AutoRoutines.Element1.position5Balance.rotationConstants,
                                AutoBalanceDirection.Backward);

                addAutoCommand(
                                Constants.AutoRoutines.Element1.position6.pathName,
                                Constants.AutoRoutines.Element1.position6.firstPathConstraint,
                                Constants.AutoRoutines.Element1.position6.remainingPathConstraints,
                                Constants.AutoRoutines.Element1.position6.translationConstants,
                                Constants.AutoRoutines.Element1.position6.rotationConstants,
                                AutoBalanceDirection.Backward);

                addAutoCommand(
                                Constants.AutoRoutines.Element2.position2.pathName,
                                Constants.AutoRoutines.Element2.position2.firstPathConstraint,
                                Constants.AutoRoutines.Element2.position2.remainingPathConstraints,
                                Constants.AutoRoutines.Element2.position2.translationConstants,
                                Constants.AutoRoutines.Element2.position2.rotationConstants,
                                Commands.none());

                addAutoCommand(
                                Constants.AutoRoutines.Element2.position8.pathName,
                                Constants.AutoRoutines.Element2.position8.firstPathConstraint,
                                Constants.AutoRoutines.Element2.position8.remainingPathConstraints,
                                Constants.AutoRoutines.Element2.position8.translationConstants,
                                Constants.AutoRoutines.Element2.position8.rotationConstants,
                                Commands.none());

                addAutoCommand(
                                Constants.AutoRoutines.Element3.position2.pathName,
                                Constants.AutoRoutines.Element3.position2.firstPathConstraint,
                                Constants.AutoRoutines.Element3.position2.remainingPathConstraints,
                                Constants.AutoRoutines.Element3.position2.translationConstants,
                                Constants.AutoRoutines.Element3.position2.rotationConstants,
                                Commands.none());

                addAutoCommand(
                                Constants.AutoRoutines.Element3.position8.pathName,
                                Constants.AutoRoutines.Element3.position8.firstPathConstraint,
                                Constants.AutoRoutines.Element3.position8.remainingPathConstraints,
                                Constants.AutoRoutines.Element3.position8.translationConstants,
                                Constants.AutoRoutines.Element3.position8.rotationConstants,
                                Commands.none());

                addAutoCommand(
                                Constants.AutoRoutines.Element1.position5Cone1.pathName,
                                Constants.AutoRoutines.Element1.position5Cone.pathName,
                                Constants.AutoRoutines.Element1.position5Cone.firstPathConstraint,
                                Constants.AutoRoutines.Element1.position5Cone.remainingPathConstraints,
                                Constants.AutoRoutines.Element1.position5Cone.translationConstants,
                                Constants.AutoRoutines.Element1.position5Cone.rotationConstants,
                                Constants.AutoRoutines.Element1.position5Cone1.pathName,
                                Constants.AutoRoutines.Element1.position5Cone1.firstPathConstraint,
                                Constants.AutoRoutines.Element1.position5Cone1.remainingPathConstraints,
                                Constants.AutoRoutines.Element1.position5Cone1.translationConstants,
                                Constants.AutoRoutines.Element1.position5Cone1.rotationConstants,
                                new Pose2d(
                                                5.27,
                                                2.75,
                                                Rotation2d.fromDegrees(180)),
                                AutoBalanceDirection.Backward);

                addAutoCommand(
                                Constants.AutoRoutines.Element1.position5Cone2.pathName,
                                Constants.AutoRoutines.Element1.position5Cone.pathName,
                                Constants.AutoRoutines.Element1.position5Cone.firstPathConstraint,
                                Constants.AutoRoutines.Element1.position5Cone.remainingPathConstraints,
                                Constants.AutoRoutines.Element1.position5Cone.translationConstants,
                                Constants.AutoRoutines.Element1.position5Cone.rotationConstants,
                                Constants.AutoRoutines.Element1.position5Cone2.pathName,
                                Constants.AutoRoutines.Element1.position5Cone2.firstPathConstraint,
                                Constants.AutoRoutines.Element1.position5Cone2.remainingPathConstraints,
                                Constants.AutoRoutines.Element1.position5Cone2.translationConstants,
                                Constants.AutoRoutines.Element1.position5Cone2.rotationConstants,
                                new Pose2d(
                                                5.27,
                                                2.75,
                                                Rotation2d.fromDegrees(180)),
                                AutoBalanceDirection.Backward);

                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void addAutoCommand(
                        String autoName,
                        String pathName,
                        PathConstraints firstPathConstraint,
                        PathConstraints[] remainingPathConstraints,
                        PIDConstants translationConstants,
                        PIDConstants rotationConstants,
                        String pathName2,
                        PathConstraints firstPathConstraint2,
                        PathConstraints[] remainingPathConstraints2,
                        PIDConstants translationConstants2,
                        PIDConstants rotationConstants2,
                        Pose2d resetPose,
                        AutoBalanceDirection direction) {

                Command balance = swerveDrive.getBalanceTestingCommand();
                Command danceParty = ledLighting.getDanceParty();
                Command balanceAndDance = Commands.parallel(balance, danceParty);

                List<PathPlannerTrajectory> trajectories = PathPlannerToAuto
                                .getPathPlannerTrajectory(
                                                pathName,
                                                firstPathConstraint,
                                                remainingPathConstraints);

                Command pathPlannerCommand = PathPlannerToAuto.createFullAutoFromPathGroup(
                                swerveDrive,
                                telemetry,
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                ledLighting,
                                trajectories,
                                translationConstants,
                                rotationConstants);

                List<PathPlannerTrajectory> trajectories2 = PathPlannerToAuto
                                .getPathPlannerTrajectory(
                                                pathName2,
                                                firstPathConstraint2,
                                                remainingPathConstraints2);

                Command pathPlannerCommand2 = PathPlannerToAuto.createFullAutoFromPathGroup(
                                swerveDrive,
                                telemetry,
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                ledLighting,
                                trajectories2,
                                translationConstants2,
                                rotationConstants2);

                CommandBase resetTelemetry = new CommandBase() {
                        @Override
                        public void execute() {
                                double[] positionFromTrackingCamera = telemetry.getTrackingCamera()
                                                .getFieldPosition(DriverStation.getAlliance());
                                Pose2d position = new Pose2d(
                                                positionFromTrackingCamera[0],
                                                positionFromTrackingCamera[1],
                                                Rotation2d.fromDegrees(positionFromTrackingCamera[5]));
                                if (positionFromTrackingCamera[0] == 0) {
                                        telemetry.resetSwerveDrivePosition(resetPose);
                                } else {
                                        telemetry.resetSwerveDrivePosition(position);
                                }
                        }

                        @Override
                        public boolean isFinished() {
                                return true;
                        }
                };

                resetTelemetry.addRequirements(swerveDrive);

                if (direction == AutoBalanceDirection.Forward) {
                        Command driveABit = swerveDrive.getOnRampCommand();
                        Command endCommand = Commands.sequence(driveABit, balanceAndDance);
                        Command autoCommand = Commands.sequence(pathPlannerCommand, resetTelemetry, pathPlannerCommand2,
                                        endCommand);
                        this.pathPlannerTrajectories.put(autoName, trajectories);
                        this.autoCommands.put(autoName, autoCommand);
                        this.autoChooser.addOption(autoName, autoName);

                } else {
                        Command driveABit = swerveDrive.getOnRampBackwardCommand();
                        Command endCommand = Commands.sequence(driveABit, balanceAndDance);
                        Command autoCommand = Commands.sequence(pathPlannerCommand, resetTelemetry, pathPlannerCommand2,
                                        endCommand);
                        this.pathPlannerTrajectories.put(autoName, trajectories);
                        this.autoCommands.put(autoName, autoCommand);
                        this.autoChooser.addOption(autoName, autoName);
                }
        }

        private void addAutoCommand(
                        String pathName,
                        PathConstraints firstPathConstraint,
                        PathConstraints[] remainingPathConstraints,
                        PIDConstants translationConstants,
                        PIDConstants rotationConstants,
                        AutoBalanceDirection direction) {
                Command balance = swerveDrive.getBalanceTestingCommand();
                Command danceParty = ledLighting.getDanceParty();
                Command balanceAndDance = Commands.parallel(balance, danceParty);

                if (direction == AutoBalanceDirection.Forward) {
                        Command driveABit = swerveDrive.getOnRampCommand();
                        Command end = Commands.sequence(driveABit, balanceAndDance);
                        addAutoCommand(pathName, firstPathConstraint, remainingPathConstraints, translationConstants,
                                        rotationConstants, end);
                } else {
                        Command driveABit = swerveDrive.getOnRampBackwardCommand();
                        Command end = Commands.sequence(driveABit, balanceAndDance);
                        addAutoCommand(pathName, firstPathConstraint, remainingPathConstraints, translationConstants,
                                        rotationConstants, end);
                }

        }

        public void addAutoCommand(
                        String pathName,
                        PathConstraints firstPathConstraint,
                        PathConstraints[] remainingPathConstraints,
                        PIDConstants translationConstants,
                        PIDConstants rotationsConstants,
                        Command endCommand) {

                List<PathPlannerTrajectory> trajectories = PathPlannerToAuto
                                .getPathPlannerTrajectory(
                                                pathName,
                                                firstPathConstraint,
                                                remainingPathConstraints);

                Command pathPlannerCommand = PathPlannerToAuto.createFullAutoFromPathGroup(
                                swerveDrive,
                                telemetry,
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                ledLighting,
                                trajectories,
                                translationConstants,
                                rotationsConstants);

                Command autoCommand = Commands.sequence(pathPlannerCommand, endCommand);

                this.pathPlannerTrajectories.put(pathName, trajectories);
                this.autoCommands.put(pathName, autoCommand);
                this.autoChooser.addOption(pathName, pathName);
        }

        public Command getAutonomousCommand() {
                if (this.autoChooser.getSelected() != null) {
                        String autoName = this.autoChooser.getSelected();
                        Command autoCommand = autoCommands.get(autoName);
                        List<PathPlannerTrajectory> autoTrajectories = pathPlannerTrajectories.get(autoName);
                        List<Pose2d> autoPoses = PathPlannerToAuto.getTrajectoryPoses(autoTrajectories);

                        fieldMap.addTrajectoryToField("Trajectory", autoPoses);
                        return autoCommand;
                } else {
                        return null;

                }
        }

        public void clearFieldMap() {
                this.fieldMap.removeTrajectoryFromField("Trajectory");
        }

        public void setSwerveModuleEncoders() {
                swerveDrive.setModuleEncoders();
        }
}
