// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommandFactories;
import frc.robot.commands.DriveAutoRoutines;
import frc.robot.commands.FullAutoRoutines;
import frc.robot.framework.controls.ControllerAxis;
import frc.robot.framework.controls.HalfControllerAxis;
import frc.robot.framework.controls.ManualChassisSpeedControl;
import frc.robot.framework.controls.ManualSpeedControl;
import frc.robot.framework.controls.PIDControllerFactory;
import frc.robot.framework.imus.Gyroscope;
import frc.robot.framework.imus.NavXGyroscopeFactory;
import frc.robot.framework.kinematics.KinematicsFactory;
import frc.robot.framework.mechanisms.LinearMech;
import frc.robot.framework.mechanisms.RotationMech;
import frc.robot.framework.mechanismsAdvanced.DualRotationMechWithLimitSwitch;
import frc.robot.framework.mechanismsAdvanced.SwerveModule;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.motors.SparkMaxFactory;
import frc.robot.framework.pneumatics.DarthMaulCylinder;
import frc.robot.framework.pneumatics.JediCylinder;
import frc.robot.framework.sensors.AngularPositionSensor;
import frc.robot.framework.sensors.CANCoderFactory;
import frc.robot.framework.sensors.LinearPositionSensor;
import frc.robot.framework.sensors.TimeOfFlightFactory;
import frc.robot.framework.sensors.WPI_TimeOfFlightFactory;
import frc.robot.framework.sensors.WPI_TimeOfFlightFactory.WPI_TimeOfFlight;
import frc.robot.framework.switches.LimitSwitch;
import frc.robot.framework.telemetry.FieldMap;
import frc.robot.framework.telemetry.SwervePoseEstimatorFactory;
import frc.robot.framework.telemetry.Telemetry;
import frc.robot.framework.vision.DumbOldCamera;
import frc.robot.framework.vision.TrackingCamera;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTilt;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.LEDLighting;
import frc.robot.subsystems.SwerveDrive;

import java.text.DecimalFormat;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        private final CommandXboxController driveController = new CommandXboxController(
                        OperatorConstants.DriverController.kDriverControllerPort);
        private final CommandXboxController operatorController = new CommandXboxController(
                        OperatorConstants.DriverController.kOperatorControllerPort);
        private final FieldMap fieldMap;
        private final RotationMech frontLeftSteeringMech;
        private final RotationMech frontRightSteeringMech;
        private final RotationMech rearLeftSteeringMech;
        private final RotationMech rearRightSteeringMech;
        private final RotationMech leftGripperMech;
        private final RotationMech rightGripperMech;
        private final LinearMech frontLeftDriveMech;
        private final LinearMech frontRightDriveMech;
        private final LinearMech rearLeftDriveMech;
        private final LinearMech rearRightDriveMech;
        private final AngularPositionSensor frontLeftSensor;
        private final AngularPositionSensor frontRightSensor;
        private final AngularPositionSensor rearLeftSensor;
        private final AngularPositionSensor rearRightSensor;
        private final AngularPositionSensor wristSensor;
        private final LinearPositionSensor elevatorPositionSensor;
        private final LimitSwitch grabberLimitSwitch;
        private final Gyroscope gyroscope;
        private final SwerveModule frontLeftModule;
        private final SwerveModule frontRightModule;
        private final SwerveModule rearLeftModule;
        private final SwerveModule rearRightModule;
        private final Telemetry telemetry;

        private final DarthMaulCylinder elevatorTilt;

        private final SwerveDrive swerveDrive;
        private final LinearMech elevatorMech;
        private final RotationMech elbowMech;

        private final JediCylinder grabberClaw;

        private final Claw claw;
        private final Grabber grabber;
        private final Elevator elevator;
        private final Elbow elbow;
        private final ElevatorTilt tilt;

        private final BooleanSupplier elbowManualControlSupplier;

        private final LEDLighting ledLighting;
        private final HashMap<String, Command> autoCommands;
        private final HashMap<String, List<PathPlannerTrajectory>> pathPlannerTrajectories;
        private final SendableChooser<String> autoChooser = new SendableChooser<>();
        private final BooleanSupplier elevatorManualControlSupplier;
        private final BooleanSupplier grabberManualControlSupplier;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                DumbOldCamera dumbOldCamera = new DumbOldCamera();

                CANSparkMax frontLeftSteeringMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.MotorConfig.deviceId,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.MotorConfig.revMotor,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.MotorConfig.isInverted,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.MotorConfig.idleMode,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.MotorConfig.velocityKP,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.MotorConfig.velocityKI,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.MotorConfig.velocityKD,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.MotorConfig.velocityKF);
                CANSparkMax frontLeftDriveMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Drive.Modules.FrontLeft.DriveMech.MotorConfig.deviceId,
                                Constants.Robot.Drive.Modules.FrontLeft.DriveMech.MotorConfig.revMotor,
                                Constants.Robot.Drive.Modules.FrontLeft.DriveMech.MotorConfig.isInverted,
                                Constants.Robot.Drive.Modules.FrontLeft.DriveMech.MotorConfig.idleMode,
                                Constants.Robot.Drive.Modules.FrontLeft.DriveMech.MotorConfig.velocityKP,
                                Constants.Robot.Drive.Modules.FrontLeft.DriveMech.MotorConfig.velocityKI,
                                Constants.Robot.Drive.Modules.FrontLeft.DriveMech.MotorConfig.velocityKD,
                                Constants.Robot.Drive.Modules.FrontLeft.DriveMech.MotorConfig.velocityKF);
                CANSparkMax frontRightSteerMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.MotorConfig.deviceId,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.MotorConfig.revMotor,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.MotorConfig.isInverted,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.MotorConfig.idleMode,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.MotorConfig.velocityKP,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.MotorConfig.velocityKI,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.MotorConfig.velocityKD,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.MotorConfig.velocityKF);
                CANSparkMax frontRightDriveMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Drive.Modules.FrontRight.DriveMech.MotorConfig.deviceId,
                                Constants.Robot.Drive.Modules.FrontRight.DriveMech.MotorConfig.revMotor,
                                Constants.Robot.Drive.Modules.FrontRight.DriveMech.MotorConfig.isInverted,
                                Constants.Robot.Drive.Modules.FrontRight.DriveMech.MotorConfig.idleMode,
                                Constants.Robot.Drive.Modules.FrontRight.DriveMech.MotorConfig.velocityKP,
                                Constants.Robot.Drive.Modules.FrontRight.DriveMech.MotorConfig.velocityKI,
                                Constants.Robot.Drive.Modules.FrontRight.DriveMech.MotorConfig.velocityKD,
                                Constants.Robot.Drive.Modules.FrontRight.DriveMech.MotorConfig.velocityKF);
                CANSparkMax rearLeftSteerMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.MotorConfig.deviceId,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.MotorConfig.revMotor,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.MotorConfig.isInverted,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.MotorConfig.idleMode,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.MotorConfig.velocityKP,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.MotorConfig.velocityKI,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.MotorConfig.velocityKD,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.MotorConfig.velocityKF);
                CANSparkMax rearLeftDriveMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Drive.Modules.RearLeft.DriveMech.MotorConfig.deviceId,
                                Constants.Robot.Drive.Modules.RearLeft.DriveMech.MotorConfig.revMotor,
                                Constants.Robot.Drive.Modules.RearLeft.DriveMech.MotorConfig.isInverted,
                                Constants.Robot.Drive.Modules.RearLeft.DriveMech.MotorConfig.idleMode,
                                Constants.Robot.Drive.Modules.RearLeft.DriveMech.MotorConfig.velocityKP,
                                Constants.Robot.Drive.Modules.RearLeft.DriveMech.MotorConfig.velocityKI,
                                Constants.Robot.Drive.Modules.RearLeft.DriveMech.MotorConfig.velocityKD,
                                Constants.Robot.Drive.Modules.RearLeft.DriveMech.MotorConfig.velocityKF);
                CANSparkMax rearRightSteerMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.MotorConfig.deviceId,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.MotorConfig.revMotor,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.MotorConfig.isInverted,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.MotorConfig.idleMode,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.MotorConfig.velocityKP,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.MotorConfig.velocityKI,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.MotorConfig.velocityKD,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.MotorConfig.velocityKF);
                CANSparkMax rearRightDriveMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Drive.Modules.RearRight.DriveMech.MotorConfig.deviceId,
                                Constants.Robot.Drive.Modules.RearRight.DriveMech.MotorConfig.revMotor,
                                Constants.Robot.Drive.Modules.RearRight.DriveMech.MotorConfig.isInverted,
                                Constants.Robot.Drive.Modules.RearRight.DriveMech.MotorConfig.idleMode,
                                Constants.Robot.Drive.Modules.RearRight.DriveMech.MotorConfig.velocityKP,
                                Constants.Robot.Drive.Modules.RearRight.DriveMech.MotorConfig.velocityKI,
                                Constants.Robot.Drive.Modules.RearRight.DriveMech.MotorConfig.velocityKD,
                                Constants.Robot.Drive.Modules.RearRight.DriveMech.MotorConfig.velocityKF);
                CANSparkMax elevatorMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Arm.Elevator.MotorConfig.deviceId,
                                Constants.Robot.Arm.Elevator.MotorConfig.revMotor,
                                Constants.Robot.Arm.Elevator.MotorConfig.isInverted,
                                Constants.Robot.Arm.Elevator.MotorConfig.idleMode,
                                Constants.Robot.Arm.Elevator.MotorConfig.velocityKP,
                                Constants.Robot.Arm.Elevator.MotorConfig.velocityKI,
                                Constants.Robot.Arm.Elevator.MotorConfig.velocityKD,
                                Constants.Robot.Arm.Elevator.MotorConfig.velocityKF);
                CANSparkMax wristMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Arm.ElbowConstants.MotorConfig.deviceId,
                                Constants.Robot.Arm.ElbowConstants.MotorConfig.revMotor,
                                Constants.Robot.Arm.ElbowConstants.MotorConfig.isInverted,
                                Constants.Robot.Arm.ElbowConstants.MotorConfig.idleMode,
                                Constants.Robot.Arm.ElbowConstants.MotorConfig.velocityKP,
                                Constants.Robot.Arm.ElbowConstants.MotorConfig.velocityKI,
                                Constants.Robot.Arm.ElbowConstants.MotorConfig.velocityKD,
                                Constants.Robot.Arm.ElbowConstants.MotorConfig.velocityKF);
                CANSparkMax leftGripperMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.deviceId,
                                Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.revMotor,
                                Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.isInverted,
                                Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.idleMode,
                                Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.velocityKP,
                                Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.velocityKI,
                                Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.velocityKD,
                                Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.velocityKF);
                CANSparkMax rightGripperMotorSparkMax = SparkMaxFactory.create(
                                Constants.Robot.Arm.Claw.RightClaw.MotorConfig.deviceId,
                                Constants.Robot.Arm.Claw.RightClaw.MotorConfig.revMotor,
                                Constants.Robot.Arm.Claw.RightClaw.MotorConfig.isInverted,
                                Constants.Robot.Arm.Claw.RightClaw.MotorConfig.idleMode,
                                Constants.Robot.Arm.Claw.RightClaw.MotorConfig.velocityKP,
                                Constants.Robot.Arm.Claw.RightClaw.MotorConfig.velocityKI,
                                Constants.Robot.Arm.Claw.RightClaw.MotorConfig.velocityKD,
                                Constants.Robot.Arm.Claw.RightClaw.MotorConfig.velocityKF);

                Motor frontLeftSteeringMotor = Motor.create(
                                frontLeftSteeringMotorSparkMax,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.MotorConfig.revMotor);
                Motor frontLeftDriveMotor = Motor.create(
                                frontLeftDriveMotorSparkMax,
                                Constants.Robot.Drive.Modules.FrontLeft.DriveMech.MotorConfig.revMotor);
                Motor frontRightSteerMotor = Motor.create(
                                frontRightSteerMotorSparkMax,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.MotorConfig.revMotor);
                Motor frontRightDriveMotor = Motor.create(
                                frontRightDriveMotorSparkMax,
                                Constants.Robot.Drive.Modules.FrontRight.DriveMech.MotorConfig.revMotor);
                Motor rearLeftSteerMotor = Motor.create(
                                rearLeftSteerMotorSparkMax,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.MotorConfig.revMotor);
                Motor rearLeftDriveMotor = Motor.create(
                                rearLeftDriveMotorSparkMax,
                                Constants.Robot.Drive.Modules.RearLeft.DriveMech.MotorConfig.revMotor);
                Motor rearRightSteerMotor = Motor.create(
                                rearRightSteerMotorSparkMax,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.MotorConfig.revMotor);
                Motor rearRightDriveMotor = Motor.create(
                                rearRightDriveMotorSparkMax,
                                Constants.Robot.Drive.Modules.RearRight.DriveMech.MotorConfig.revMotor);
                Motor elevatorMotor = Motor.create(
                                elevatorMotorSparkMax,
                                Constants.Robot.Arm.Elevator.MotorConfig.revMotor);
                Motor wristMotor = Motor.create(
                                wristMotorSparkMax,
                                Constants.Robot.Arm.ElbowConstants.MotorConfig.revMotor);
                Motor leftGripperMotor = Motor.create(
                                leftGripperMotorSparkMax,
                                Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.revMotor);
                Motor rightGripperMotor = Motor.create(
                                rightGripperMotorSparkMax,
                                Constants.Robot.Arm.Claw.RightClaw.MotorConfig.revMotor);

                WPI_CANCoder frontLeftCANCoder = CANCoderFactory.create(
                                Constants.Robot.Drive.Modules.FrontLeft.AngleSensor.SensorConfig.deviceId,
                                Constants.Robot.Drive.Modules.FrontLeft.AngleSensor.SensorConfig.canbus,
                                frontLeftSteeringMotor,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.motorToMechConversion);

                WPI_CANCoder frontRightCANCoder = CANCoderFactory.create(
                                Constants.Robot.Drive.Modules.FrontRight.AngleSensor.SensorConfig.deviceId,
                                Constants.Robot.Drive.Modules.FrontRight.AngleSensor.SensorConfig.canbus,
                                frontRightSteerMotor,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.motorToMechConversion);

                WPI_CANCoder rearLeftCANCoder = CANCoderFactory.create(
                                Constants.Robot.Drive.Modules.RearLeft.AngleSensor.SensorConfig.deviceId,
                                Constants.Robot.Drive.Modules.RearLeft.AngleSensor.SensorConfig.canbus,
                                rearLeftSteerMotor,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.motorToMechConversion);

                WPI_CANCoder rearRightCANCoder = CANCoderFactory.create(
                                Constants.Robot.Drive.Modules.RearRight.AngleSensor.SensorConfig.deviceId,
                                Constants.Robot.Drive.Modules.RearRight.AngleSensor.SensorConfig.canbus,
                                rearRightSteerMotor,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.motorToMechConversion);

                this.frontLeftSensor = AngularPositionSensor.create(frontLeftCANCoder);

                this.frontRightSensor = AngularPositionSensor.create(frontRightCANCoder);

                this.rearLeftSensor = AngularPositionSensor.create(rearLeftCANCoder);

                this.rearRightSensor = AngularPositionSensor.create(rearRightCANCoder);

                this.frontLeftSteeringMech = RotationMech.create(
                                frontLeftSteeringMotor,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.motorToMechConversion,
                                frontLeftSensor);

                this.frontRightSteeringMech = RotationMech.create(
                                frontRightSteerMotor,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.motorToMechConversion,
                                frontRightSensor);

                this.rearLeftSteeringMech = RotationMech.create(
                                rearLeftSteerMotor,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.motorToMechConversion,
                                rearLeftSensor);

                this.rearRightSteeringMech = RotationMech.create(
                                rearRightSteerMotor,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.motorToMechConversion,
                                rearRightSensor);

                this.leftGripperMech = RotationMech.create(
                                leftGripperMotor,
                                Constants.Robot.Arm.Claw.LeftClaw.MechConfig.motorToMechConversion);

                this.rightGripperMech = RotationMech.create(
                                rightGripperMotor,
                                Constants.Robot.Arm.Claw.RightClaw.MechConfig.motorToMechConversion);

                this.frontLeftDriveMech = LinearMech.create(
                                frontLeftDriveMotor,
                                Constants.Robot.Drive.Modules.driveMotorMotorToRotationMechConversion,
                                Constants.Robot.Drive.Modules.driveMotorRotationToLinearConversion);

                this.frontRightDriveMech = LinearMech.create(
                                frontRightDriveMotor,
                                Constants.Robot.Drive.Modules.driveMotorMotorToRotationMechConversion,
                                Constants.Robot.Drive.Modules.driveMotorRotationToLinearConversion);

                this.rearLeftDriveMech = LinearMech.create(
                                rearLeftDriveMotor,
                                Constants.Robot.Drive.Modules.driveMotorMotorToRotationMechConversion,
                                Constants.Robot.Drive.Modules.driveMotorRotationToLinearConversion);

                this.rearRightDriveMech = LinearMech.create(
                                rearRightDriveMotor,
                                Constants.Robot.Drive.Modules.driveMotorMotorToRotationMechConversion,
                                Constants.Robot.Drive.Modules.driveMotorRotationToLinearConversion);

                TimeOfFlight elevatorTimeOfFlight = TimeOfFlightFactory.create(
                                Constants.Robot.Arm.Elevator.Sensor.positionSensorID,
                                Constants.Robot.Arm.Elevator.Sensor.mode,
                                Constants.Robot.Arm.Elevator.Sensor.sampleTime);

                WPI_TimeOfFlight elevatorWPI_TimeOfFlight = WPI_TimeOfFlightFactory.create(
                                elevatorTimeOfFlight,
                                Constants.Robot.Arm.Elevator.Sensor.positionSensorID,
                                elevatorMotor,
                                Constants.Robot.Arm.Elevator.MechConfig.motorToMechConversion,
                                Constants.Robot.Arm.Elevator.MechConfig.rotationToLinearconversion,
                                Constants.Robot.Arm.Elevator.Control.minPosition,
                                Constants.Robot.Arm.Elevator.Control.maxPosition);

                this.elevatorPositionSensor = LinearPositionSensor.create(
                                Constants.Robot.Arm.Elevator.Sensor.positionSensorID,
                                elevatorWPI_TimeOfFlight);

                TimeOfFlight grabberTimeOfFlight = TimeOfFlightFactory.create(
                                Constants.Robot.Arm.Claw.LimitSwitch.Sensor.positionSensorID,
                                Constants.Robot.Arm.Claw.LimitSwitch.Sensor.mode,
                                Constants.Robot.Arm.Claw.LimitSwitch.Sensor.sampleTime);

                WPI_TimeOfFlight grabberWPI_TimeOfFlight = WPI_TimeOfFlightFactory.create(
                                grabberTimeOfFlight,
                                Constants.Robot.Arm.Claw.LimitSwitch.Sensor.positionSensorID);

                LinearPositionSensor grabberTimeOfFlightSensor = LinearPositionSensor.create(
                                Constants.Robot.Arm.Claw.LimitSwitch.Sensor.positionSensorID,
                                grabberWPI_TimeOfFlight);

                this.grabberLimitSwitch = LimitSwitch.create(
                                grabberTimeOfFlightSensor,
                                Constants.Robot.Arm.Claw.LimitSwitch.minTruePositionMeters,
                                Constants.Robot.Arm.Claw.LimitSwitch.maxTruePositionMeters);

                this.frontLeftModule = new SwerveModule(
                                frontLeftSteeringMech,
                                frontLeftDriveMech);

                this.frontRightModule = new SwerveModule(
                                frontRightSteeringMech,
                                frontRightDriveMech);

                this.rearLeftModule = new SwerveModule(
                                rearLeftSteeringMech,
                                rearLeftDriveMech);

                this.rearRightModule = new SwerveModule(
                                rearRightSteeringMech,
                                rearRightDriveMech);

                Pose2d initialPosition = new Pose2d(3, 3, new Rotation2d());

                SwerveDriveKinematics swerveDriveKinematics = KinematicsFactory.createSwerveKinematics(
                                Constants.Robot.Drive.Modules.FrontLeft.location,
                                Constants.Robot.Drive.Modules.FrontRight.location,
                                Constants.Robot.Drive.Modules.RearLeft.location,
                                Constants.Robot.Drive.Modules.RearRight.location);

                SwerveDrivePoseEstimator swerveDrivePoseEstimator = SwervePoseEstimatorFactory.create(
                                swerveDriveKinematics,
                                new Rotation2d(),
                                initialPosition,
                                this.frontLeftModule,
                                this.frontRightModule,
                                this.rearLeftModule,
                                this.rearRightModule);

                this.telemetry = Telemetry.create(
                                swerveDrivePoseEstimator,
                                this.frontLeftModule,
                                this.frontRightModule,
                                this.rearLeftModule,
                                this.rearRightModule);

                AHRS navXMXP2 = NavXGyroscopeFactory.create(
                                Constants.Robot.Drive.Gyroscope.serial_port_id,
                                swerveDriveKinematics,
                                this.frontLeftModule,
                                this.frontRightModule,
                                this.rearLeftModule,
                                this.rearRightModule);

                this.gyroscope = Gyroscope.create(navXMXP2);

                ControllerAxis fieldCentricForwardBackAxis = ControllerAxis.getAxisControl(
                                driveController,
                                XboxController.Axis.kLeftY,
                                true,
                                Constants.OperatorConstants.DriverController.kDeadband);

                ControllerAxis fieldCentricLeftRightAxis = ControllerAxis.getAxisControl(
                                driveController,
                                XboxController.Axis.kLeftX,
                                true,
                                Constants.OperatorConstants.DriverController.kDeadband);

                ControllerAxis robotCentricForwardBackAxis = ControllerAxis.getAxisControl(
                                driveController,
                                XboxController.Axis.kRightY,
                                true,
                                Constants.OperatorConstants.DriverController.kDeadband);

                ControllerAxis robotCentricLeftRightAxis = ControllerAxis.getAxisControl(
                                driveController,
                                XboxController.Axis.kRightX,
                                true,
                                Constants.OperatorConstants.DriverController.kDeadband);

                HalfControllerAxis rotationPositiveAxis = HalfControllerAxis.getAxisControl(
                                driveController,
                                HalfControllerAxis.PositiveOnlyAxisType.LeftTriggerAxis,
                                false,
                                Constants.OperatorConstants.DriverController.kDeadband);

                HalfControllerAxis rotationNegativeAxis = HalfControllerAxis.getAxisControl(
                                driveController,
                                HalfControllerAxis.PositiveOnlyAxisType.RightTriggerAxis,
                                false,
                                Constants.OperatorConstants.DriverController.kDeadband);

                ControllerAxis rotationAxis = ControllerAxis.getAxisControl(rotationPositiveAxis, rotationNegativeAxis);

                ManualSpeedControl fieldCentricForwardBackControl = new ManualSpeedControl(
                                fieldCentricForwardBackAxis,
                                Constants.OperatorConstants.DriverController.maxTranslationalSpeedMPS);

                ManualSpeedControl fieldCentricLeftRightControl = new ManualSpeedControl(
                                fieldCentricLeftRightAxis,
                                Constants.OperatorConstants.DriverController.maxTranslationalSpeedMPS);

                ManualSpeedControl robotCentricForwardBackControl = new ManualSpeedControl(
                                robotCentricForwardBackAxis,
                                Constants.OperatorConstants.DriverController.maxTranslationalSpeedMPS);

                ManualSpeedControl robotCentricLeftRightControl = new ManualSpeedControl(
                                robotCentricLeftRightAxis,
                                Constants.OperatorConstants.DriverController.maxTranslationalSpeedMPS);

                ManualSpeedControl rotationControl = new ManualSpeedControl(
                                rotationAxis,
                                Constants.OperatorConstants.DriverController.maxRotationalSpeedRadPS);

                ManualChassisSpeedControl manualChassisSpeedControl = ManualChassisSpeedControl
                                .getManualChassisSpeedControl(
                                                fieldCentricForwardBackControl,
                                                fieldCentricLeftRightControl,
                                                robotCentricForwardBackControl,
                                                robotCentricLeftRightControl,
                                                rotationControl);

                PIDController frontLeftSteerPIDController = PIDControllerFactory.create(
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.kP,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.kI,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.kD,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.tolerance,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.minimumInput,
                                Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.maximumInput);

                PIDController frontRightSteerPIDController = PIDControllerFactory.create(
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.kP,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.kI,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.kD,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.tolerance,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.minimumInput,
                                Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.maximumInput);

                PIDController rearLeftSteerPIDController = PIDControllerFactory.create(
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.kP,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.kI,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.kD,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.tolerance,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.minimumInput,
                                Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.maximumInput);

                PIDController rearRightSteerPIDController = PIDControllerFactory.create(
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.kP,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.kI,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.kD,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.tolerance,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.minimumInput,
                                Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.maximumInput);

                this.fieldMap = new FieldMap();

                TrackingCamera trackingCamera = TrackingCamera.createFromLimeLight("limelight");

                this.swerveDrive = new SwerveDrive(
                                frontLeftModule,
                                frontRightModule,
                                rearLeftModule,
                                rearRightModule,
                                gyroscope,
                                telemetry,
                                trackingCamera,
                                swerveDriveKinematics,
                                manualChassisSpeedControl,
                                frontLeftSteerPIDController,
                                frontRightSteerPIDController,
                                rearLeftSteerPIDController,
                                rearRightSteerPIDController,
                                fieldMap);

                ControllerAxis elevatorUpDownAxis = ControllerAxis.getAxisControl(
                                operatorController,
                                XboxController.Axis.kLeftY,
                                true,
                                Constants.OperatorConstants.ElevatorManualControls.kDeadband);

                ManualSpeedControl elevatorManualSpeedControl = new ManualSpeedControl(
                                elevatorUpDownAxis,
                                Constants.Robot.Arm.Elevator.Control.maxManualSpeedMPS);

                ControllerAxis armUpDownAxis = ControllerAxis.getAxisControl(
                                operatorController,
                                XboxController.Axis.kRightY,
                                false,
                                Constants.OperatorConstants.ElevatorManualControls.kDeadband);

                ManualSpeedControl elbowManualSpeedControl = new ManualSpeedControl(
                                armUpDownAxis,
                                Constants.Robot.Arm.ElbowConstants.Control.maxManualSpeedRotationsPerSecond);

                this.elbowManualControlSupplier = new BooleanSupplier() {

                        @Override
                        public boolean getAsBoolean() {
                                return elbowManualSpeedControl.getManualSpeed() != 0;
                        }

                };

                this.elevatorManualControlSupplier = new BooleanSupplier() {

                        @Override
                        public boolean getAsBoolean() {
                                return elevatorManualSpeedControl.getManualSpeed() != 0;
                        }

                };

                this.elevatorMech = LinearMech.create(
                                elevatorMotor,
                                Constants.Robot.Arm.Elevator.MechConfig.motorToMechConversion,
                                Constants.Robot.Arm.Elevator.MechConfig.rotationToLinearconversion,
                                elevatorPositionSensor);

                this.wristSensor = AngularPositionSensor.getREVThroughBoreEncoder(
                                Constants.Robot.Arm.ElbowConstants.Sensor.dioChannel,
                                Constants.Robot.Arm.ElbowConstants.Sensor.offsetDegrees,
                                wristMotor,
                                Constants.Robot.Arm.ElbowConstants.MechConfig.motorToMechConversion);

                HalfControllerAxis grabberPositiveRotation = HalfControllerAxis.getAxisControl(
                                operatorController,
                                HalfControllerAxis.PositiveOnlyAxisType.LeftTriggerAxis,
                                false,
                                Constants.OperatorConstants.ElevatorManualControls.kDeadband);

                HalfControllerAxis grabberNegativeRotation = HalfControllerAxis.getAxisControl(
                                operatorController,
                                HalfControllerAxis.PositiveOnlyAxisType.RightTriggerAxis,
                                false,
                                Constants.OperatorConstants.ElevatorManualControls.kDeadband);

                ControllerAxis grabberControlAxis = ControllerAxis.getAxisControl(grabberPositiveRotation,
                                grabberNegativeRotation);

                ManualSpeedControl grabberManualControl = new ManualSpeedControl(
                                grabberControlAxis,
                                Constants.Robot.Arm.Claw.Control.maxManualSpeedRPS);

                this.grabberManualControlSupplier = new BooleanSupplier() {

                        @Override
                        public boolean getAsBoolean() {
                                return grabberManualControl.getManualSpeed() != 0;
                        }

                };

                this.elbowMech = RotationMech.create(
                                wristMotor,
                                Constants.Robot.Arm.ElbowConstants.MechConfig.motorToMechConversion,
                                wristSensor);

                DualRotationMechWithLimitSwitch grabberMech = DualRotationMechWithLimitSwitch.create(
                                leftGripperMech,
                                rightGripperMech,
                                grabberLimitSwitch);

                DoubleSolenoid grabberClawDoubleSolenoid = new DoubleSolenoid(
                                Constants.Robot.Arm.Pneumatics.pneumaticsModuleID,
                                PneumaticsModuleType.REVPH,
                                Constants.Robot.Arm.Claw.Control.doubleSolenoidChannel1,
                                Constants.Robot.Arm.Claw.Control.doubleSolenoidChannel2);

                Solenoid tiltSolenoid1 = new Solenoid(
                                Constants.Robot.Arm.Pneumatics.pneumaticsModuleID,
                                PneumaticsModuleType.REVPH,
                                Constants.Robot.Arm.Elevator.Tilt.solenoid1Channel);

                Solenoid tiltSolenoid2 = new Solenoid(
                                Constants.Robot.Arm.Pneumatics.pneumaticsModuleID,
                                PneumaticsModuleType.REVPH,
                                Constants.Robot.Arm.Elevator.Tilt.solenoid2Channel);

                this.grabberClaw = JediCylinder.getDoubleSolenoidBased(grabberClawDoubleSolenoid);

                JediCylinder frontCylinder = JediCylinder.getSingleSolenoidBased(tiltSolenoid1);

                JediCylinder backCylinder = JediCylinder.getSingleSolenoidBased(tiltSolenoid2);

                this.elevatorTilt = new DarthMaulCylinder(
                                frontCylinder,
                                backCylinder);
                PIDController elevatorPIDController = PIDControllerFactory.create(
                                Constants.Robot.Arm.Elevator.Control.PID.kP,
                                Constants.Robot.Arm.Elevator.Control.PID.kI,
                                Constants.Robot.Arm.Elevator.Control.PID.kD,
                                Constants.Robot.Arm.Elevator.Control.PID.tolerance);

                PIDController elbowPIDController = PIDControllerFactory.create(
                                Constants.Robot.Arm.ElbowConstants.Control.PID.kP,
                                Constants.Robot.Arm.ElbowConstants.Control.PID.kI,
                                Constants.Robot.Arm.ElbowConstants.Control.PID.kD,
                                Constants.Robot.Arm.ElbowConstants.Control.PID.tolerance,
                                Constants.Robot.Arm.ElbowConstants.Control.PID.minimumInput,
                                Constants.Robot.Arm.ElbowConstants.Control.PID.maximumInput);

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

                this.tilt = new ElevatorTilt(elevatorTilt, fakeElevatorMech2d, elevatorMech2d);

                this.claw = new Claw(grabberClaw);

                DecimalFormat decimalFormat = new DecimalFormat("###.###");

                this.grabber = new Grabber(grabberMech, grabberManualControl, decimalFormat);
                this.grabber.setDefaultCommand(this.grabber.createStopCommand());
                this.elevator = new Elevator(
                                elevatorMech,
                                elevatorManualSpeedControl,
                                elevatorPIDController,
                                Constants.Robot.Arm.Elevator.Control.minPosition,
                                Constants.Robot.Arm.Elevator.Control.maxPosition,
                                elevatorMech2d,
                                decimalFormat);

                CommandBase elevatorHoldCommand = this.elevator.createHoldCommand();
                this.elevator.setDefaultCommand(elevatorHoldCommand);

                this.elbow = new Elbow(
                                elbowMech,
                                elbowManualSpeedControl,
                                elbowPIDController,
                                Constants.Robot.Arm.ElbowConstants.Control.minAngleDegrees,
                                Constants.Robot.Arm.ElbowConstants.Control.maxAngleDegrees,
                                elbowMech2d,
                                decimalFormat);

                CommandBase elbowHoldCommand = this.elbow.createHoldCommand();
                this.elbow.setDefaultCommand(elbowHoldCommand);

                this.ledLighting = new LEDLighting(
                                17,
                                "CANIVORE",
                                300);

                // List<PathPlannerTrajectory> Element1DockPosition1Trajectory =
                // FullAutoRoutines.getPathPlannerTrajectory(
                // Constants.AutoRoutines.Element1.position1.pathName,
                // Constants.AutoRoutines.Element1.position1.firstPathConstraint,
                // Constants.AutoRoutines.Element1.position1.remainingPathConstraints);

                // Command Element1DockPosition1Command =
                // FullAutoRoutines.createFullAutoFromPathGroup(
                // swerveDrive,
                // elevator,
                // elbow,
                // grabber,
                // claw,
                // tilt,
                // ledLighting,
                // Element1DockPosition1Trajectory,
                // Constants.AutoRoutines.Element1.position1.translationConstants,
                // Constants.AutoRoutines.Element1.position1.rotationConstants);

                // Command driveForwardABit = swerveDrive.getOnRampCommand();
                // Command balance = swerveDrive.getBalanceCommand();
                // Element1DockPosition1Command =
                // Commands.sequence(Element1DockPosition1Command, driveForwardABit, balance);

                List<PathPlannerTrajectory> Element2DockPosition2Trajectory = FullAutoRoutines.getPathPlannerTrajectory(
                                Constants.AutoRoutines.Element2.position2.pathName,
                                Constants.AutoRoutines.Element2.position2.firstPathConstraint,
                                Constants.AutoRoutines.Element2.position2.remainingPathConstraints);

                Command Element2DockPosition2Command = FullAutoRoutines.createFullAutoFromPathGroup(
                                swerveDrive,
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                ledLighting,
                                Element2DockPosition2Trajectory,
                                Constants.AutoRoutines.Element2.position2.translationConstants,
                                Constants.AutoRoutines.Element2.position2.rotationConstants);

                // List<PathPlannerTrajectory> Element1DockPosition3Trajectory =
                // FullAutoRoutines.getPathPlannerTrajectory(
                // Constants.AutoRoutines.Element1.position3.pathName,
                // Constants.AutoRoutines.Element1.position3.firstPathConstraint,
                // Constants.AutoRoutines.Element1.position3.remainingPathConstraints);

                // Command Element1DockPosition3Command =
                // FullAutoRoutines.createFullAutoFromPathGroup(
                // swerveDrive,
                // elevator,
                // elbow,
                // grabber,
                // claw,
                // tilt,
                // ledLighting,
                // Element1DockPosition3Trajectory,
                // Constants.AutoRoutines.Element1.position3.translationConstants,
                // Constants.AutoRoutines.Element1.position3.rotationConstants);

                List<PathPlannerTrajectory> Element1DockPosition4Trajectory = FullAutoRoutines.getPathPlannerTrajectory(
                                Constants.AutoRoutines.Element1.position4.pathName,
                                Constants.AutoRoutines.Element1.position4.firstPathConstraint,
                                Constants.AutoRoutines.Element1.position4.remainingPathConstraints);

                Command Element1DockPosition4Command = FullAutoRoutines.createFullAutoFromPathGroup(
                                swerveDrive,
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                ledLighting,
                                Element1DockPosition4Trajectory,
                                Constants.AutoRoutines.Element1.position4.translationConstants,
                                Constants.AutoRoutines.Element1.position4.rotationConstants);

                Command balance14 = swerveDrive.getBalanceCommand();
                Element1DockPosition4Command = Commands.sequence(Element1DockPosition4Command, balance14);

                List<PathPlannerTrajectory> Element1DockPosition5Trajectory = FullAutoRoutines.getPathPlannerTrajectory(
                                Constants.AutoRoutines.Element1.position5.pathName,
                                Constants.AutoRoutines.Element1.position5.firstPathConstraint,
                                Constants.AutoRoutines.Element1.position5.remainingPathConstraints);

                Command Element1DockPosition5Command = FullAutoRoutines.createFullAutoFromPathGroup(
                                swerveDrive,
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                ledLighting,
                                Element1DockPosition5Trajectory,
                                Constants.AutoRoutines.Element1.position5.translationConstants,
                                Constants.AutoRoutines.Element1.position5.rotationConstants);

                Command balance15 = swerveDrive.getBalanceCommand();
                Element1DockPosition5Command = Commands.sequence(Element1DockPosition5Command, balance15);

                List<PathPlannerTrajectory> Element1DockPosition6Trajectory = FullAutoRoutines.getPathPlannerTrajectory(
                                Constants.AutoRoutines.Element1.position6.pathName,
                                Constants.AutoRoutines.Element1.position6.firstPathConstraint,
                                Constants.AutoRoutines.Element1.position6.remainingPathConstraints);

                Command Element1DockPosition6Command = FullAutoRoutines.createFullAutoFromPathGroup(
                                swerveDrive,
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                ledLighting,
                                Element1DockPosition6Trajectory,
                                Constants.AutoRoutines.Element1.position6.translationConstants,
                                Constants.AutoRoutines.Element1.position6.rotationConstants);

                Command balance16 = swerveDrive.getBalanceCommand();
                Element1DockPosition6Command = Commands.sequence(Element1DockPosition6Command, balance16);

                // List<PathPlannerTrajectory> Element1DockPosition7Trajectory =
                // FullAutoRoutines.getPathPlannerTrajectory(
                // Constants.AutoRoutines.Element1.position7.pathName,
                // Constants.AutoRoutines.Element1.position7.firstPathConstraint,
                // Constants.AutoRoutines.Element1.position7.remainingPathConstraints);

                // Command Element1DockPosition7Command =
                // FullAutoRoutines.createFullAutoFromPathGroup(
                // swerveDrive,
                // elevator,
                // elbow,
                // grabber,
                // claw,
                // tilt,
                // ledLighting,
                // Element1DockPosition7Trajectory,
                // Constants.AutoRoutines.Element1.position7.translationConstants,
                // Constants.AutoRoutines.Element1.position7.rotationConstants);

                List<PathPlannerTrajectory> Element2DockPosition8Trajectory = FullAutoRoutines.getPathPlannerTrajectory(
                                Constants.AutoRoutines.Element2.position8.pathName,
                                Constants.AutoRoutines.Element2.position8.firstPathConstraint,
                                Constants.AutoRoutines.Element2.position8.remainingPathConstraints);

                Command Element2DockPosition8Command = FullAutoRoutines.createFullAutoFromPathGroup(
                                swerveDrive,
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                ledLighting,
                                Element2DockPosition8Trajectory,
                                Constants.AutoRoutines.Element2.position8.translationConstants,
                                Constants.AutoRoutines.Element2.position8.rotationConstants);

                List<PathPlannerTrajectory> Element3DockPosition2Trajectory = FullAutoRoutines.getPathPlannerTrajectory(
                                Constants.AutoRoutines.Element3.position2.pathName,
                                Constants.AutoRoutines.Element3.position2.firstPathConstraint,
                                Constants.AutoRoutines.Element3.position2.remainingPathConstraints);

                Command Element3DockPosition2Command = FullAutoRoutines.createFullAutoFromPathGroup(
                                swerveDrive,
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                ledLighting,
                                Element3DockPosition2Trajectory,
                                Constants.AutoRoutines.Element3.position2.translationConstants,
                                Constants.AutoRoutines.Element3.position2.rotationConstants);

                List<PathPlannerTrajectory> Element3DockPosition8Trajectory = FullAutoRoutines.getPathPlannerTrajectory(
                                Constants.AutoRoutines.Element3.position8.pathName,
                                Constants.AutoRoutines.Element3.position8.firstPathConstraint,
                                Constants.AutoRoutines.Element3.position8.remainingPathConstraints);

                Command Element3DockPosition8Command = FullAutoRoutines.createFullAutoFromPathGroup(
                                swerveDrive,
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                ledLighting,
                                Element3DockPosition8Trajectory,
                                Constants.AutoRoutines.Element3.position8.translationConstants,
                                Constants.AutoRoutines.Element3.position8.rotationConstants);

                // List<PathPlannerTrajectory> Element1DockPosition9Trajectory =
                // FullAutoRoutines.getPathPlannerTrajectory(
                // Constants.AutoRoutines.Element1.position9.pathName,
                // Constants.AutoRoutines.Element1.position9.firstPathConstraint,
                // Constants.AutoRoutines.Element1.position9.remainingPathConstraints);

                // Command Element1DockPosition9Command =
                // FullAutoRoutines.createFullAutoFromPathGroup(
                // swerveDrive,
                // elevator,
                // elbow,
                // grabber,
                // claw,
                // tilt,
                // ledLighting,
                // Element1DockPosition9Trajectory,
                // Constants.AutoRoutines.Element1.position9.translationConstants,
                // Constants.AutoRoutines.Element1.position9.rotationConstants);

                // List<PathPlannerTrajectory> Element2DockPosition1Trajectory =
                // FullAutoRoutines.getPathPlannerTrajectory(
                // Constants.AutoRoutines.Element2.position1.pathName,
                // Constants.AutoRoutines.Element2.position1.firstPathConstraint,
                // Constants.AutoRoutines.Element2.position1.remainingPathConstraints);

                // Command Element2DockPosition1Command =
                // FullAutoRoutines.createFullAutoFromPathGroup(
                // swerveDrive,
                // elevator,
                // elbow,
                // grabber,
                // claw,
                // tilt,
                // ledLighting,
                // Element2DockPosition1Trajectory,
                // Constants.AutoRoutines.Element2.position1.translationConstants,
                // Constants.AutoRoutines.Element2.position1.rotationConstants);

                // List<PathPlannerTrajectory> Element2DockPosition2Trajectory =
                // FullAutoRoutines.getPathPlannerTrajectory(
                // Constants.AutoRoutines.Element2.position2.pathName,
                // Constants.AutoRoutines.Element2.position2.firstPathConstraint,
                // Constants.AutoRoutines.Element2.position2.remainingPathConstraints);

                // Command Element2DockPosition2Command =
                // FullAutoRoutines.createFullAutoFromPathGroup(
                // swerveDrive,
                // elevator,
                // elbow,
                // grabber,
                // claw,
                // tilt,
                // ledLighting,
                // Element2DockPosition2Trajectory,
                // Constants.AutoRoutines.Element2.position2.translationConstants,
                // Constants.AutoRoutines.Element2.position2.rotationConstants);

                BooleanSupplier ledPurpleBooleanSupplier = new BooleanSupplier() {

                        @Override
                        public boolean getAsBoolean() {
                                return claw.getState() == Claw.State.CUBE;
                        }

                };

                BooleanSupplier ledGoldBooleanSupplier = new BooleanSupplier() {

                        @Override
                        public boolean getAsBoolean() {
                                return claw.getState() == Claw.State.CONE;
                        }

                };

                Trigger ledPurpleTrigger = new Trigger(ledPurpleBooleanSupplier);
                Trigger ledGoldTrigger = new Trigger(ledGoldBooleanSupplier);

                CommandBase ledPurpleCommand = ledLighting.getSetLEDCommand(
                                0X3C,
                                0X09,
                                0X49);

                CommandBase ledGoldCommand = ledLighting.getSetLEDCommand(
                                0XFF,
                                0XD7,
                                0X00);

                ledGoldTrigger.onTrue(ledGoldCommand);
                ledPurpleTrigger.onTrue(ledPurpleCommand);

                pathPlannerTrajectories = new HashMap<>();
                // pathPlannerTrajectories.put("1ElementPosition1",
                // Element1DockPosition1Trajectory);
                // pathPlannerTrajectories.put("1ElementPosition2",
                // Element1DockPosition2Trajectory);
                // pathPlannerTrajectories.put("1ElementPosition3",
                // Element1DockPosition3Trajectory);
                pathPlannerTrajectories.put("1ElementPosition4", Element1DockPosition4Trajectory);
                pathPlannerTrajectories.put("1ElementPosition5", Element1DockPosition5Trajectory);
                pathPlannerTrajectories.put("1ElementPosition6", Element1DockPosition6Trajectory);
                // pathPlannerTrajectories.put("1ElementPosition7",
                // Element1DockPosition7Trajectory);
                // pathPlannerTrajectories.put("1ElementPosition8",
                // Element1DockPosition8Trajectory);
                // pathPlannerTrajectories.put("1ElementPosition9",
                // Element1DockPosition9Trajectory);
                pathPlannerTrajectories.put("2ElementPosition2", Element2DockPosition2Trajectory);
                pathPlannerTrajectories.put("2ElementPosition8", Element2DockPosition8Trajectory);
                pathPlannerTrajectories.put("3ElementPosition2", Element3DockPosition2Trajectory);
                pathPlannerTrajectories.put("3ElementPosition8", Element3DockPosition8Trajectory);

                autoCommands = new HashMap<>();
                // autoCommands.put("1ElementPosition1", Element1DockPosition1Command);
                // autoCommands.put("1ElementPosition2", Element1DockPosition2Command);
                // autoCommands.put("1ElementPosition3", Element1DockPosition3Command);
                autoCommands.put("1ElementPosition4", Element1DockPosition4Command);
                autoCommands.put("1ElementPosition5", Element1DockPosition5Command);
                autoCommands.put("1ElementPosition6", Element1DockPosition6Command);
                // autoCommands.put("1ElementPosition7", Element1DockPosition7Command);
                // autoCommands.put("1ElementPosition8", Element1DockPosition8Command);
                // autoCommands.put("1ElementPosition9", Element1DockPosition9Command);
                autoCommands.put("2ElementPosition2", Element2DockPosition2Command);
                autoCommands.put("2ElementPosition8", Element2DockPosition8Command);
                autoCommands.put("3ElementPosition2", Element3DockPosition2Command);
                autoCommands.put("3ElementPosition8", Element3DockPosition8Command);

                // this.autoChooser.addOption("1ElementPosition1", "1ElementPosition1");
                // this.autoChooser.addOption("1ElementPosition2", "1ElementPosition2");
                // this.autoChooser.addOption("1ElementPosition3", "1ElementPosition3");
                this.autoChooser.addOption("1ElementPosition4", "1ElementPosition4");
                this.autoChooser.addOption("1ElementPosition5", "1ElementPosition5");
                this.autoChooser.addOption("1ElementPosition6", "1ElementPosition6");
                // this.autoChooser.addOption("1ElementPosition7", "1ElementPosition7");
                // this.autoChooser.addOption("1ElementPosition8", "1ElementPosition8");
                // this.autoChooser.addOption("1ElementPosition9", "1ElementPosition9");
                this.autoChooser.addOption("2ElementPosition2", "2ElementPosition2");
                this.autoChooser.addOption("2ElementPosition8", "2ElementPosition8");
                this.autoChooser.addOption("3ElementPosition2", "3ElementPosition2");
                this.autoChooser.addOption("3ElementPosition8", "3ElementPosition8");

                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putString("RIO Serial Number", RoboRioDataJNI.getSerialNumber());

                // Configure the trigger bindings
                configureBindings();
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {
                configureDriverBindings();
                configureOperatorBindings();
        }

        private void configureDriverBindings() {

                CommandBase zeroModulesCommand = swerveDrive.getZeroModuleCommand();
                CommandBase setTelemetryFromCameraCommand = swerveDrive.setTelemetryFromCameraCommand();
                CommandBase dropPortalAlign = DriveAutoRoutines.create(
                                swerveDrive,
                                Constants.Auto.Drive.PortalPositions.dropPortal,
                                fieldMap);

                CommandBase leftPortalAlign = DriveAutoRoutines.create(
                                swerveDrive,
                                Constants.Auto.Drive.PortalPositions.leftPortal,
                                fieldMap);

                CommandBase rightPortalAlign = DriveAutoRoutines.create(
                                swerveDrive,
                                Constants.Auto.Drive.PortalPositions.rightPortal,
                                fieldMap);

                CommandBase nearestScoreAlign = swerveDrive.getOnTheFlyDriveCommand(
                                Constants.Auto.Drive.ScoringPositions.positionsList,
                                fieldMap);

                // INFO: Zero Wheels Command
                driveController.back().whileTrue(zeroModulesCommand);

                // INFO: Reset Telemetry From Safe Known Position
                driveController.start().onTrue(setTelemetryFromCameraCommand);

                // INFO: Align Low
                driveController.b().onTrue(ArmCommandFactories.Alignment.createLow(elevator, elbow, tilt, claw));

                // INFO: Align Middle
                driveController.x().onTrue(ArmCommandFactories.Alignment.createMiddle(elevator, elbow, tilt, claw));

                // INFO: Align High
                driveController.y().onTrue(ArmCommandFactories.Alignment.createHigh(elevator, elbow, tilt, claw));

                // INFO: Score Low
                driveController.a()
                                .onTrue(ArmCommandFactories.Placement.createLow(elevator, elbow, tilt, claw, grabber));

                // INFO: Score Middle
                driveController.leftBumper()
                                .onTrue(ArmCommandFactories.Placement.createMiddle(elevator, elbow, tilt, claw,
                                                grabber));

                // INFO: Score High
                driveController.rightBumper()
                                .onTrue(ArmCommandFactories.Placement.createHigh(elevator, elbow, tilt, claw, grabber));

                // INFO: Test Targeting
                driveController.povLeft().whileTrue(leftPortalAlign);

                // INFO: Toggle Limelight Vision Adding
                driveController.povUp().whileTrue(nearestScoreAlign);

                // INFO: Reset Known
                driveController.povRight().whileTrue(rightPortalAlign);

                // INFO: Auto Balance
                driveController.povDown().whileTrue(dropPortalAlign);

        }

        private void configureOperatorBindings() {

                CommandBase balanceTest = swerveDrive.getBalanceCommand();

                CommandBase elementCarry = ArmCommandFactories.Carry.create(
                                elevator,
                                elbow,
                                grabber,
                                tilt,
                                Constants.Auto.Arm.Carry.elevatorPositionMeters,
                                Constants.Auto.Arm.Carry.elbowPositionDegrees,
                                Constants.Auto.Arm.Carry.elevatorTiltState);

                CommandBase conePickupFloor = ArmCommandFactories.Pickup.create(
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

                CommandBase cubePickupFloor = ArmCommandFactories.Pickup.create(
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

                CommandBase conePickupSliding = ArmCommandFactories.Pickup.create(
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                Constants.Auto.Arm.Pickup.Sliding.elevatorPositionMeters,
                                Constants.Auto.Arm.Pickup.Sliding.elbowPositionDegrees,
                                Constants.Auto.Arm.Pickup.grabberSpeedRPM,
                                Constants.Auto.Arm.Pickup.Sliding.elevatorTiltState,
                                Claw.State.CONE);

                CommandBase cubePickupSliding = ArmCommandFactories.Pickup.create(
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                Constants.Auto.Arm.Pickup.Sliding.elevatorPositionCubeMeters,
                                Constants.Auto.Arm.Pickup.Sliding.elbowPositionDegrees,
                                Constants.Auto.Arm.Pickup.grabberSpeedRPM,
                                Constants.Auto.Arm.Pickup.Sliding.elevatorTiltState,
                                Claw.State.CUBE);

                CommandBase conePickupDrop = ArmCommandFactories.Pickup.create(
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                Constants.Auto.Arm.Pickup.Drop.elevatorPositionMeters,
                                Constants.Auto.Arm.Pickup.Drop.Cone.elbowPositionDegrees,
                                Constants.Auto.Arm.Pickup.grabberSpeedRPM,
                                Constants.Auto.Arm.Pickup.Drop.elevatorTiltState,
                                Claw.State.CONE);

                CommandBase cubePickupDrop = ArmCommandFactories.Pickup.create(
                                elevator,
                                elbow,
                                grabber,
                                claw,
                                tilt,
                                Constants.Auto.Arm.Pickup.Drop.elevatorPositionMeters,
                                Constants.Auto.Arm.Pickup.Drop.Cube.elbowPositionDegrees,
                                Constants.Auto.Arm.Pickup.grabberSpeedRPM,
                                Constants.Auto.Arm.Pickup.Drop.elevatorTiltState,
                                Claw.State.CUBE);

                CommandBase grabberToggleCommand = claw.createSetStateCommand();

                CommandBase noLegsCommand = tilt.createSetStateCommand(ElevatorTilt.State.NONE);
                CommandBase shortSaber = tilt.createSetStateCommand(ElevatorTilt.State.TWO);
                CommandBase tatooine = tilt.createSetStateCommand(ElevatorTilt.State.SIX);
                CommandBase duelOfTheFates = tilt.createSetStateCommand(ElevatorTilt.State.EIGHT);

                Trigger elbowManualControlTrigger = new Trigger(elbowManualControlSupplier);
                Trigger elevatorManualControlTrigger = new Trigger(elevatorManualControlSupplier);
                Trigger grabberManualControlTrigger = new Trigger(grabberManualControlSupplier);

                CommandBase elbowManualControlCommand = elbow.createControlCommand();
                CommandBase elevatorManualControlCommand = elevator.createControlCommand();
                CommandBase grabberManualControlCommand = grabber.createControlCommand();

                elbowManualControlTrigger.whileTrue(elbowManualControlCommand);
                elevatorManualControlTrigger.whileTrue(elevatorManualControlCommand);
                grabberManualControlTrigger.whileTrue(grabberManualControlCommand);

                // INFO: Manual Pickup Commands
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

                // INFO: Manual Grabber Toggle Commands
                operatorController.back().onTrue(grabberToggleCommand);

                // INFO: Manual Tilt Commands
                operatorController.povDown().onTrue(noLegsCommand);
                operatorController.povLeft().onTrue(shortSaber);
                operatorController.povRight().onTrue(tatooine);
                operatorController.povUp().onTrue(duelOfTheFates);

                operatorController.start().whileTrue(balanceTest);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                if (this.autoChooser.getSelected() != null) {
                        String autoName = this.autoChooser.getSelected();
                        Command autoCommand = autoCommands.get(autoName);
                        List<PathPlannerTrajectory> autoTrajectories = pathPlannerTrajectories.get(autoName);
                        List<Pose2d> autoPoses = FullAutoRoutines.getTrajectoryPoses(autoTrajectories);

                        fieldMap.addTrajectoryToField("Trajectory", autoPoses);
                        return autoCommand;
                } else {
                        return null;

                }
        }

        public void clearFieldMap() {
                this.fieldMap.removeTrajectoryFromField("Trajectory");
        }
}
