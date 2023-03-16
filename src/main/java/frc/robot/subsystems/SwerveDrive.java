// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
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
import frc.robot.framework.mechanismsAdvanced.SwerveModule;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.motors.SparkMaxFactory;
import frc.robot.framework.sensors.AngularPositionSensor;
import frc.robot.framework.sensors.CANCoderFactory;
import frc.robot.framework.telemetry.FieldMap;
import frc.robot.framework.telemetry.SwervePoseEstimatorFactory;
import frc.robot.framework.telemetry.FieldTelemetry;
import frc.robot.framework.vision.TrackingCamera;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule rearLeftModule;
  private final SwerveModule rearRightModule;
  private final ManualChassisSpeedControl manualChassisSpeedControl;
  private final PIDController frontLeftSteerPIDController;
  private final PIDController frontRightSteerPIDController;
  private final PIDController rearLeftSteerPIDController;
  private final PIDController rearRightSteerPIDController;
  private final PIDController forwardController;
  private final PIDController strafeController;
  private final PIDController rotationController;
  private final SwerveDriveKinematics swerveDriveKinematics;

  public SwerveDrive(
      CommandXboxController driveController,
      FieldMap fieldMap,
      SwerveDriveKinematics swerveDriveKinematics) {

    this.forwardController = new PIDController(5, 0, 0);
    this.strafeController = new PIDController(5, 0, 0);
    this.rotationController = new PIDController(5, 0, 0);

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

    AngularPositionSensor frontLeftSensor = AngularPositionSensor.create(frontLeftCANCoder);

    AngularPositionSensor frontRightSensor = AngularPositionSensor.create(frontRightCANCoder);

    AngularPositionSensor rearLeftSensor = AngularPositionSensor.create(rearLeftCANCoder);

    AngularPositionSensor rearRightSensor = AngularPositionSensor.create(rearRightCANCoder);

    RotationMech frontLeftSteeringMech = RotationMech.create(
        frontLeftSteeringMotor,
        Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.motorToMechConversion,
        frontLeftSensor);

    RotationMech frontRightSteeringMech = RotationMech.create(
        frontRightSteerMotor,
        Constants.Robot.Drive.Modules.FrontRight.SteeringMech.motorToMechConversion,
        frontRightSensor);

    RotationMech rearLeftSteeringMech = RotationMech.create(
        rearLeftSteerMotor,
        Constants.Robot.Drive.Modules.RearLeft.SteeringMech.motorToMechConversion,
        rearLeftSensor);

    RotationMech rearRightSteeringMech = RotationMech.create(
        rearRightSteerMotor,
        Constants.Robot.Drive.Modules.RearRight.SteeringMech.motorToMechConversion,
        rearRightSensor);

    LinearMech frontLeftDriveMech = LinearMech.create(
        frontLeftDriveMotor,
        Constants.Robot.Drive.Modules.driveMotorMotorToRotationMechConversion,
        Constants.Robot.Drive.Modules.driveMotorRotationToLinearConversion);

    LinearMech frontRightDriveMech = LinearMech.create(
        frontRightDriveMotor,
        Constants.Robot.Drive.Modules.driveMotorMotorToRotationMechConversion,
        Constants.Robot.Drive.Modules.driveMotorRotationToLinearConversion);

    LinearMech rearLeftDriveMech = LinearMech.create(
        rearLeftDriveMotor,
        Constants.Robot.Drive.Modules.driveMotorMotorToRotationMechConversion,
        Constants.Robot.Drive.Modules.driveMotorRotationToLinearConversion);

    LinearMech rearRightDriveMech = LinearMech.create(
        rearRightDriveMotor,
        Constants.Robot.Drive.Modules.driveMotorMotorToRotationMechConversion,
        Constants.Robot.Drive.Modules.driveMotorRotationToLinearConversion);

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

    this.manualChassisSpeedControl = ManualChassisSpeedControl
        .getManualChassisSpeedControl(
            fieldCentricForwardBackControl,
            fieldCentricLeftRightControl,
            robotCentricForwardBackControl,
            robotCentricLeftRightControl,
            rotationControl);

    this.frontLeftSteerPIDController = PIDControllerFactory.create(
        Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.kP,
        Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.kI,
        Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.kD,
        Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.tolerance,
        Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.minimumInput,
        Constants.Robot.Drive.Modules.FrontLeft.SteeringMech.PID.maximumInput);

    this.frontRightSteerPIDController = PIDControllerFactory.create(
        Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.kP,
        Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.kI,
        Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.kD,
        Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.tolerance,
        Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.minimumInput,
        Constants.Robot.Drive.Modules.FrontRight.SteeringMech.PID.maximumInput);

    this.rearLeftSteerPIDController = PIDControllerFactory.create(
        Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.kP,
        Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.kI,
        Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.kD,
        Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.tolerance,
        Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.minimumInput,
        Constants.Robot.Drive.Modules.RearLeft.SteeringMech.PID.maximumInput);

    this.rearRightSteerPIDController = PIDControllerFactory.create(
        Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.kP,
        Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.kI,
        Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.kD,
        Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.tolerance,
        Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.minimumInput,
        Constants.Robot.Drive.Modules.RearRight.SteeringMech.PID.maximumInput);

    this.swerveDriveKinematics = swerveDriveKinematics;

  }

  @Override
  public void periodic() {

    // ChassisSpeeds fieldCentricChassisSpeedsFromController =
    // manualChassisSpeedControl
    // .getFieldCentricSpeeds(telemetry.getCurrentPosition().getRotation());
    // ChassisSpeeds robotCentricChassisSpeedsFromController =
    // manualChassisSpeedControl
    // .getRobotCentricSpeeds(telemetry.getCurrentPosition().getRotation());

    // ChassisSpeeds robotCentricChassisSpeedsActual = getSwerveDriveChassisSpeed();
    // ChassisSpeeds fieldCentricChassisSpeedsActual =
    // ChassisSpeeds.fromFieldRelativeSpeeds(
    // robotCentricChassisSpeedsActual,
    // telemetry.getCurrentPosition().getRotation().unaryMinus());

    // SmartDashboard.putNumber("FieldCentricVXFromController",
    // fieldCentricChassisSpeedsFromController.vxMetersPerSecond);
    // SmartDashboard.putNumber("FieldCentricVYFromController",
    // fieldCentricChassisSpeedsFromController.vyMetersPerSecond);
    // SmartDashboard.putNumber("FieldCentricWFromController",
    // fieldCentricChassisSpeedsFromController.omegaRadiansPerSecond);

    // SmartDashboard.putNumber("RobotCentricVXFromController",
    // robotCentricChassisSpeedsFromController.vxMetersPerSecond);
    // SmartDashboard.putNumber("RobotCentricVYFromController",
    // robotCentricChassisSpeedsFromController.vyMetersPerSecond);
    // SmartDashboard.putNumber("RobotCentricWFromController",
    // robotCentricChassisSpeedsFromController.omegaRadiansPerSecond);

    // SmartDashboard.putNumber("FieldCentricVXActual",
    // fieldCentricChassisSpeedsActual.vxMetersPerSecond);
    // SmartDashboard.putNumber("FieldCentricVYActual",
    // fieldCentricChassisSpeedsActual.vyMetersPerSecond);
    // SmartDashboard.putNumber("FieldCentricWActual",
    // fieldCentricChassisSpeedsActual.omegaRadiansPerSecond);

    // SmartDashboard.putNumber("RobotCentricVXActual",
    // robotCentricChassisSpeedsActual.vxMetersPerSecond);
    // SmartDashboard.putNumber("RobotCentricVYActual",
    // robotCentricChassisSpeedsActual.vyMetersPerSecond);
    // SmartDashboard.putNumber("RobotCentricWActual",
    // robotCentricChassisSpeedsActual.omegaRadiansPerSecond);

    // SmartDashboard.putNumber("RobotFieldPositionX",
    // this.telemetry.getCurrentPosition().getX());
    // SmartDashboard.putNumber("RobotFieldPositionY",
    // this.telemetry.getCurrentPosition().getY());

    // SmartDashboard.putNumber("RobotFieldPositionYawDeg",
    // this.telemetry.getCurrentPosition().getRotation().getDegrees());

  }

  // Functions
  private Rotation2d getSteeringSpeed(
      Rotation2d desiredAngle,
      Rotation2d currentAngle,
      PIDController pidController) {

    double desiredAngleValue = desiredAngle.getRotations();
    desiredAngleValue %= 1;
    desiredAngleValue = desiredAngleValue < 0 ? desiredAngleValue + 1 : desiredAngleValue;

    pidController.setSetpoint(desiredAngleValue);
    double rotationsPerSecond = pidController.calculate(currentAngle.getRotations());
    Rotation2d mechRotationsPerSecond = Rotation2d.fromRotations(rotationsPerSecond);

    return mechRotationsPerSecond;

  }

  // Suppliers
  public ChassisSpeeds getSwerveDriveChassisSpeed() {

    SwerveModuleState frontLeftModuleState = this.frontLeftModule.getSwerveModuleState();
    SwerveModuleState frontRightModuleState = this.frontRightModule.getSwerveModuleState();
    SwerveModuleState rearLeftModuleState = this.rearLeftModule.getSwerveModuleState();
    SwerveModuleState rearRightModuleState = this.rearRightModule.getSwerveModuleState();

    return this.swerveDriveKinematics.toChassisSpeeds(frontLeftModuleState, frontRightModuleState, rearLeftModuleState,
        rearRightModuleState);

  }

  public SwerveModule[] getSwerveModules() {
    return new SwerveModule[] {
        this.frontLeftModule,
        this.frontRightModule,
        this.rearLeftModule,
        this.rearRightModule
    };
  }

  // Runnables
  private void stopDrive() {
    this.frontLeftModule.stop();
    this.frontRightModule.stop();
    this.rearLeftModule.stop();
    this.rearRightModule.stop();
  }

  public void setModuleEncoders() {
    this.frontLeftModule.setSwerveModuleSteeringEncoder();
    this.frontRightModule.setSwerveModuleSteeringEncoder();
    this.rearLeftModule.setSwerveModuleSteeringEncoder();
    this.rearRightModule.setSwerveModuleSteeringEncoder();
  }

  // Consumers
  public void setSwerveDriveChassisSpeed(ChassisSpeeds robotCentricChassisSpeeds) {

    if (robotCentricChassisSpeeds.vxMetersPerSecond == 0 &&
        robotCentricChassisSpeeds.vyMetersPerSecond == 0 &&
        robotCentricChassisSpeeds.omegaRadiansPerSecond == 0) {
      stopDrive();
    } else {
      SwerveModuleState[] swerveModuleStates = this.swerveDriveKinematics
          .toSwerveModuleStates(robotCentricChassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Robot.Drive.Modules.maxModuleSpeedMPS);

      SwerveModuleState frontLeftDesiredState = swerveModuleStates[0];
      Rotation2d frontLeftCurrentAngle = this.frontLeftModule.getSwerveModuleState().angle;
      frontLeftDesiredState = SwerveModuleState.optimize(frontLeftDesiredState, frontLeftCurrentAngle);
      double frontLeftDesiredDriveSpeedMPS = frontLeftDesiredState.speedMetersPerSecond;
      Rotation2d frontLeftDesiredSteerSpeedRPS = getSteeringSpeed(
          frontLeftDesiredState.angle,
          frontLeftCurrentAngle,
          frontLeftSteerPIDController);

      SwerveModuleState frontRightDesiredState = swerveModuleStates[1];
      Rotation2d frontRightCurrentAngle = this.frontRightModule.getSwerveModuleState().angle;
      frontRightDesiredState = SwerveModuleState.optimize(frontRightDesiredState, frontRightCurrentAngle);
      double frontRightDesiredDriveSpeedMPS = frontRightDesiredState.speedMetersPerSecond;
      Rotation2d frontRightDesiredSteerSpeedRPS = getSteeringSpeed(
          frontRightDesiredState.angle,
          frontRightCurrentAngle,
          frontRightSteerPIDController);

      SwerveModuleState rearLeftDesiredState = swerveModuleStates[2];
      Rotation2d rearLeftCurrentAngle = this.rearLeftModule.getSwerveModuleState().angle;
      rearLeftDesiredState = SwerveModuleState.optimize(rearLeftDesiredState, rearLeftCurrentAngle);
      double rearLeftDesiredDriveSpeedMPS = rearLeftDesiredState.speedMetersPerSecond;
      Rotation2d rearLeftDesiredSteerSpeedRPS = getSteeringSpeed(
          rearLeftDesiredState.angle,
          rearLeftCurrentAngle,
          rearLeftSteerPIDController);

      SwerveModuleState rearRightDesiredState = swerveModuleStates[3];
      Rotation2d rearRightCurrentAngle = this.rearRightModule.getSwerveModuleState().angle;
      rearRightDesiredState = SwerveModuleState.optimize(rearRightDesiredState, rearRightCurrentAngle);
      double rearRightDesiredDriveSpeedMPS = rearRightDesiredState.speedMetersPerSecond;
      Rotation2d rearRightDesiredSteerSpeedRPS = getSteeringSpeed(
          rearRightDesiredState.angle,
          rearRightCurrentAngle,
          rearRightSteerPIDController);

      this.frontLeftModule.setSwerveModuleState(frontLeftDesiredSteerSpeedRPS, frontLeftDesiredDriveSpeedMPS);
      this.frontRightModule.setSwerveModuleState(frontRightDesiredSteerSpeedRPS, frontRightDesiredDriveSpeedMPS);
      this.rearLeftModule.setSwerveModuleState(rearLeftDesiredSteerSpeedRPS, rearLeftDesiredDriveSpeedMPS);
      this.rearRightModule.setSwerveModuleState(rearRightDesiredSteerSpeedRPS, rearRightDesiredDriveSpeedMPS);
    }

  }

  public void setManualDefaultCommand() {

    CommandBase manualControlCommand = Commands.run(
        () -> {
          ChassisSpeeds chassisSpeeds = this.manualChassisSpeedControl
              .getRobotCentricSpeeds(this.telemetry.getCurrentPosition().getRotation());
          this.setSwerveDriveChassisSpeed(chassisSpeeds);
        }, this);

    this.setDefaultCommand(manualControlCommand);
  }

  public CommandBase getZeroModuleCommand() {
    return Commands.runEnd(
        () -> {

          Rotation2d frontLeftSteeringSpeed = getSteeringSpeed(
              new Rotation2d(),
              this.frontLeftModule.getSwerveModulePosition().angle,
              frontLeftSteerPIDController);

          Rotation2d frontRightSteeringSpeed = getSteeringSpeed(
              new Rotation2d(),
              this.frontRightModule.getSwerveModulePosition().angle,
              frontRightSteerPIDController);

          Rotation2d rearLeftSteeringSpeed = getSteeringSpeed(
              new Rotation2d(),
              this.rearLeftModule.getSwerveModulePosition().angle,
              rearLeftSteerPIDController);

          Rotation2d rearRightSteeringSpeed = getSteeringSpeed(
              new Rotation2d(),
              this.rearRightModule.getSwerveModulePosition().angle,
              rearRightSteerPIDController);

          this.frontLeftModule.setSwerveModuleState(frontLeftSteeringSpeed, 0);
          this.frontRightModule.setSwerveModuleState(frontRightSteeringSpeed, 0);
          this.rearLeftModule.setSwerveModuleState(rearLeftSteeringSpeed, 0);
          this.rearRightModule.setSwerveModuleState(rearRightSteeringSpeed, 0);
        },
        () -> {
          this.frontLeftModule.stop();
          this.frontRightModule.stop();
          this.rearLeftModule.stop();
          this.rearRightModule.stop();
        },
        this);
  }

  public CommandBase getOnRampCommand() {

    CommandBase wait = Commands.waitSeconds(0.5);
    CommandBase drive = Commands.run(
        () -> {
          this.setSwerveDriveChassisSpeed(new ChassisSpeeds(4 / 1.4 * Math.sin(Math.toRadians(15)), 0, 0));
        },
        this);

    return Commands.race(wait, drive);

  }

  public CommandBase getOnRampBackwardCommand() {

    CommandBase wait = Commands.waitSeconds(0.5);
    CommandBase drive = Commands.run(
        () -> {
          this.setSwerveDriveChassisSpeed(new ChassisSpeeds(-4 / 1.4 * Math.sin(Math.toRadians(15)), 0, 0));
        },
        this);

    return Commands.race(wait, drive);

  }

  public CommandBase getBalanceCommand() {
    CommandBase balance = new CommandBase() {

      @Override
      public void initialize() {
        SmartDashboard.putString("Swerve Drive Current Command", "Balancing");
      }

      @Override
      public void execute() {
        Rotation2d pitchAngle = gyroscope.getPitch();
        Rotation2d pitchDistanceFrom0 = pitchAngle.minus(new Rotation2d());
        double pitchDistanceFrom0Radians = pitchDistanceFrom0.getRadians();

        double vxMetersPerSecond = -Constants.Robot.Drive.Modules.maxModuleSpeedMPS
            * Math.sin(pitchDistanceFrom0Radians) / 2.5;

        vxMetersPerSecond = MathUtil.applyDeadband(vxMetersPerSecond, 0.10);
        setSwerveDriveChassisSpeed(new ChassisSpeeds(vxMetersPerSecond, 0, 0));
      }

      @Override
      public void end(boolean interrupted) {
        stopDrive();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };

    balance.addRequirements(this);
    return balance;
  }

  public CommandBase getBalanceTestingCommand() {
    CommandBase balance = new CommandBase() {

      @Override
      public void initialize() {
        SmartDashboard.putString("Swerve Drive Current Command", "Balancing");
      }

      @Override
      public void execute() {
        Rotation2d pitchAngle = gyroscope.getPitch();
        Rotation2d rollAngle = gyroscope.getRoll();
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
        setSwerveDriveChassisSpeed(new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, 0));
      }

      @Override
      public void end(boolean interrupted) {
        stopDrive();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };

    balance.addRequirements(this);
    return balance;
  }

  public CommandBase createSlidingPortalCommand(
      Pose2d bluePortal,
      Pose2d redPortal) {

    CommandBase leftPortalCommand = new CommandBase() {

      @Override
      public void initialize() {
        Pose2d goalPosition = bluePortal;
        if (DriverStation.getAlliance() == Alliance.Red) {
          goalPosition = redPortal;
          goalPosition = new Pose2d(goalPosition.getX(), 8.02 - goalPosition.getY(), goalPosition.getRotation());
        }
        forwardController.setSetpoint(goalPosition.getX());
        strafeController.setSetpoint(goalPosition.getY());
        rotationController.setSetpoint(goalPosition.getRotation().getRotations());
      }

      @Override
      public void execute() {
        Pose2d currentPosition = telemetry.getCurrentPosition();
        double vxMetersPerSecond = forwardController.calculate(currentPosition.getX());
        double vyMetersPerSecond = strafeController.calculate(currentPosition.getY());
        double rotationSpeedRPS = rotationController.calculate(currentPosition.getRotation().getRotations());
        double omegaRadiansPerSecond = rotationSpeedRPS * Math.PI * 2;
        ChassisSpeeds fcChassisSpeed = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        ChassisSpeeds rcChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fcChassisSpeed,
            currentPosition.getRotation());
        setSwerveDriveChassisSpeed(rcChassisSpeeds);
      }

      @Override
      public void end(boolean interrupted) {
        stopDrive();
      }

      @Override
      public boolean isFinished() {
        return forwardController.atSetpoint() && strafeController.atSetpoint() && rotationController.atSetpoint();
      }

    };

    leftPortalCommand.addRequirements(this);
    return leftPortalCommand;

  }

  public CommandBase createDropPortalCommand(Pose2d bluePortal) {

    rotationController.enableContinuousInput(0, 1);

    CommandBase leftPortalCommand = new CommandBase() {

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
        Pose2d currentPosition = telemetry.getCurrentPosition();
        double vxMetersPerSecond = forwardController.calculate(currentPosition.getX());
        double vyMetersPerSecond = strafeController.calculate(currentPosition.getY());
        double rotationSpeedRPS = rotationController.calculate(currentPosition.getRotation().getRotations());
        double omegaRadiansPerSecond = rotationSpeedRPS * Math.PI * 2;
        ChassisSpeeds fcChassisSpeed = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        ChassisSpeeds rcChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fcChassisSpeed,
            currentPosition.getRotation());
        setSwerveDriveChassisSpeed(rcChassisSpeeds);
      }

      @Override
      public void end(boolean interrupted) {
        stopDrive();
      }

      @Override
      public boolean isFinished() {
        return forwardController.atSetpoint() && strafeController.atSetpoint() && rotationController.atSetpoint();
      }

    };

    leftPortalCommand.addRequirements(this);
    return leftPortalCommand;

  }

  public CommandBase createNearestPointCommand(
      List<Pose2d> scoringPositions) {

    CommandBase leftPortalCommand = new CommandBase() {

      @Override
      public void initialize() {

        Pose2d goalPosition = getClosestPosition(scoringPositions);

        forwardController.setSetpoint(goalPosition.getX());
        strafeController.setSetpoint(goalPosition.getY());
        rotationController.setSetpoint(goalPosition.getRotation().getRotations());
      }

      @Override
      public void execute() {
        Pose2d currentPosition = telemetry.getCurrentPosition();
        double vxMetersPerSecond = forwardController.calculate(currentPosition.getX());
        double vyMetersPerSecond = strafeController.calculate(currentPosition.getY());
        double rotationSpeedRPS = rotationController.calculate(currentPosition.getRotation().getRotations());
        double omegaRadiansPerSecond = rotationSpeedRPS * Math.PI * 2;
        ChassisSpeeds fcChassisSpeed = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        ChassisSpeeds rcChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fcChassisSpeed,
            currentPosition.getRotation());
        setSwerveDriveChassisSpeed(rcChassisSpeeds);
      }

      @Override
      public void end(boolean interrupted) {
        stopDrive();
      }

      @Override
      public boolean isFinished() {
        return forwardController.atSetpoint() && strafeController.atSetpoint() && rotationController.atSetpoint();
      }

    };

    leftPortalCommand.addRequirements(this);
    return leftPortalCommand;

  }

}
