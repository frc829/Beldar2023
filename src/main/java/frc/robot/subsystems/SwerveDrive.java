// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.framework.controls.ManualChassisSpeedControl;
import frc.robot.framework.imus.Gyroscope;
import frc.robot.framework.mechanismsAdvanced.SwerveModule;
import frc.robot.framework.telemetry.FieldMap;
import frc.robot.framework.telemetry.Telemetry;
import frc.robot.framework.vision.TrackingCamera;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule rearLeftModule;
  private final SwerveModule rearRightModule;
  private final Gyroscope gyroscope;
  private final Telemetry telemetry;
  private final TrackingCamera trackingCamera;
  private final SwerveDriveKinematics swerveDriveKinematics;
  private final ManualChassisSpeedControl manualChassisSpeedControl;
  private final PIDController frontLeftSteerPIDController;
  private final PIDController frontRightSteerPIDController;
  private final PIDController rearLeftSteerPIDController;
  private final PIDController rearRightSteerPIDController;
  private final FieldMap fieldMap;

  private boolean allowVisionMeasurements = true;

  public SwerveDrive(
      SwerveModule frontLeftModule,
      SwerveModule frontRightModule,
      SwerveModule rearLeftModule,
      SwerveModule rearRightModule,
      Gyroscope gyroscope,
      Telemetry telemetry,
      TrackingCamera trackingCamera,
      SwerveDriveKinematics swerveDriveKinematics,
      ManualChassisSpeedControl manualChassisSpeedControl,
      PIDController frontLeftSteerPIDController,
      PIDController frontRightSteerPIDController,
      PIDController rearLeftSteerPIDController,
      PIDController rearRightSteerPIDController,
      FieldMap fieldMap) {

    this.frontLeftModule = frontLeftModule;
    this.frontRightModule = frontRightModule;
    this.rearLeftModule = rearLeftModule;
    this.rearRightModule = rearRightModule;

    this.gyroscope = gyroscope;
    this.telemetry = telemetry;
    this.trackingCamera = trackingCamera;
    this.swerveDriveKinematics = swerveDriveKinematics;
    this.manualChassisSpeedControl = manualChassisSpeedControl;

    this.frontLeftSteerPIDController = frontLeftSteerPIDController;
    this.frontRightSteerPIDController = frontRightSteerPIDController;
    this.rearLeftSteerPIDController = rearLeftSteerPIDController;
    this.rearRightSteerPIDController = rearRightSteerPIDController;

    this.fieldMap = fieldMap;

    this.setManualDefaultCommand();

  }

  public void toggleAllowVisionMeasurements() {
    this.allowVisionMeasurements = !this.allowVisionMeasurements;
  }

  @Override
  public void periodic() {

    this.telemetry.updateCurrentPosition(this.gyroscope.getYaw());
    double[] currentPoseFromCamera = this.trackingCamera.getFieldPosition(DriverStation.getAlliance());
    if (currentPoseFromCamera[0] != 0) {
      SmartDashboard.putBoolean("LimeLightGood", true);
    } else {
      SmartDashboard.putBoolean("LimeLightGood", false);
    }

    telemetry.addVisionMeasurement(currentPoseFromCamera);
    fieldMap.updateField(this.telemetry.getCurrentPosition());

    SmartDashboard.putBoolean("Gyro Connected", this.gyroscope.isConnected());
    SmartDashboard.putNumber("PoseFromCameraX", currentPoseFromCamera[0]);
    SmartDashboard.putNumber("PoseFromCameraY", currentPoseFromCamera[1]);
    SmartDashboard.putNumber("PoseFromCameraZ", currentPoseFromCamera[2]);
    SmartDashboard.putNumber("PoseFromCameraRoll", currentPoseFromCamera[3]);
    SmartDashboard.putNumber("PoseFromCameraPitch", currentPoseFromCamera[4]);
    SmartDashboard.putNumber("PoseFromCameraYaw", currentPoseFromCamera[5]);

    SmartDashboard.putNumber("Gyro Pitch", this.gyroscope.getPitch().getDegrees());
    SmartDashboard.putNumber("Gyro Roll", this.gyroscope.getRoll().getDegrees());
    SmartDashboard.putNumber("Gyro Yaw", this.gyroscope.getYaw().getDegrees());

    this.telemetry.updateCurrentPosition(gyroscope.getYaw());

    ChassisSpeeds fieldCentricChassisSpeedsFromController = manualChassisSpeedControl
        .getFieldCentricSpeeds(telemetry.getCurrentPosition().getRotation());
    ChassisSpeeds robotCentricChassisSpeedsFromController = manualChassisSpeedControl
        .getRobotCentricSpeeds(telemetry.getCurrentPosition().getRotation());

    ChassisSpeeds robotCentricChassisSpeedsActual = getSwerveDriveChassisSpeed();
    ChassisSpeeds fieldCentricChassisSpeedsActual = ChassisSpeeds.fromFieldRelativeSpeeds(
        robotCentricChassisSpeedsActual, telemetry.getCurrentPosition().getRotation().unaryMinus());

    SmartDashboard.putNumber("FieldCentricVXFromController", fieldCentricChassisSpeedsFromController.vxMetersPerSecond);
    SmartDashboard.putNumber("FieldCentricVYFromController", fieldCentricChassisSpeedsFromController.vyMetersPerSecond);
    SmartDashboard.putNumber("FieldCentricWFromController",
        fieldCentricChassisSpeedsFromController.omegaRadiansPerSecond);

    SmartDashboard.putNumber("RobotCentricVXFromController", robotCentricChassisSpeedsFromController.vxMetersPerSecond);
    SmartDashboard.putNumber("RobotCentricVYFromController", robotCentricChassisSpeedsFromController.vyMetersPerSecond);
    SmartDashboard.putNumber("RobotCentricWFromController",
        robotCentricChassisSpeedsFromController.omegaRadiansPerSecond);

    SmartDashboard.putNumber("FieldCentricVXActual", fieldCentricChassisSpeedsActual.vxMetersPerSecond);
    SmartDashboard.putNumber("FieldCentricVYActual", fieldCentricChassisSpeedsActual.vyMetersPerSecond);
    SmartDashboard.putNumber("FieldCentricWActual", fieldCentricChassisSpeedsActual.omegaRadiansPerSecond);

    SmartDashboard.putNumber("RobotCentricVXActual", robotCentricChassisSpeedsActual.vxMetersPerSecond);
    SmartDashboard.putNumber("RobotCentricVYActual", robotCentricChassisSpeedsActual.vyMetersPerSecond);
    SmartDashboard.putNumber("RobotCentricWActual", robotCentricChassisSpeedsActual.omegaRadiansPerSecond);

    SmartDashboard.putNumber("RobotFieldPositionX", this.telemetry.getCurrentPosition().getX());
    SmartDashboard.putNumber("RobotFieldPositionY", this.telemetry.getCurrentPosition().getY());

    SmartDashboard.putNumber("RobotFieldPositionYawDeg",
        this.telemetry.getCurrentPosition().getRotation().getDegrees());

    SmartDashboard.putNumber("GyroRoll", (this.gyroscope.getRoll().getDegrees()));
    SmartDashboard.putNumber("GyroPitch", (this.gyroscope.getPitch().getDegrees()));
    SmartDashboard.putNumber("GyroYaw", (this.gyroscope.getYaw().getDegrees()));
  }

  private void stopDrive() {
    this.frontLeftModule.stop();
    this.frontRightModule.stop();
    this.rearLeftModule.stop();
    this.rearRightModule.stop();
  }

  public Pose2d getSwerveDrivePosition() {
    return this.telemetry.getCurrentPosition();
  }

  public void resetSwerveDrivePosition(Pose2d newPose) {
    this.telemetry.resetCurrentPosition(newPose, this.gyroscope.getYaw());
  }

  public ChassisSpeeds getSwerveDriveChassisSpeed() {

    SwerveModuleState frontLeftModuleState = this.frontLeftModule.getSwerveModuleState();
    SwerveModuleState frontRightModuleState = this.frontRightModule.getSwerveModuleState();
    SwerveModuleState rearLeftModuleState = this.rearLeftModule.getSwerveModuleState();
    SwerveModuleState rearRightModuleState = this.rearRightModule.getSwerveModuleState();

    return this.swerveDriveKinematics.toChassisSpeeds(frontLeftModuleState, frontRightModuleState, rearLeftModuleState,
        rearRightModuleState);

  }

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

  public void setManualDefaultCommand() {

    CommandBase manualControlCommand = Commands.run(
        () -> {
          ChassisSpeeds chassisSpeeds = this.manualChassisSpeedControl
              .getRobotCentricSpeeds(this.telemetry.getCurrentPosition().getRotation());
          this.setSwerveDriveChassisSpeed(chassisSpeeds);
        }, this);

    this.setDefaultCommand(manualControlCommand);
  }

  public void setTelemetryFromCamera() {

    double[] currentPoseFromCamera = this.trackingCamera.getFieldPosition(DriverStation.getAlliance());
    Rotation2d yawFromCamera = Rotation2d.fromDegrees(currentPoseFromCamera[5]);
    SmartDashboard.putNumber("YawFROMCAMERA", yawFromCamera.getDegrees());
    Pose2d cameraPose = new Pose2d(currentPoseFromCamera[0], currentPoseFromCamera[1], yawFromCamera);
    this.telemetry.resetCurrentPosition(cameraPose, gyroscope.getYaw());
  }

  public CommandBase getZeroModuleCommand() {
    return Commands.runEnd(
        () -> {
          this.frontLeftModule.setSwerveModuleSteeringEncoder();
          this.frontRightModule.setSwerveModuleSteeringEncoder();
          this.rearLeftModule.setSwerveModuleSteeringEncoder();
          this.rearRightModule.setSwerveModuleSteeringEncoder();

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
        });
  }

  public CommandBase getOnTheFlyDriveCommand(Pose2d goalPose, FieldMap fieldMap) {

    SwerveDrive swerveDrive = this;

    CommandBase onTheFlyCommand = new CommandBase() {
      private PPSwerveControllerCommand drivingCommand;

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

        drivingCommand = new PPSwerveControllerCommand(
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

        fieldMap.addTrajectoryToField("Trajectory", poses);

        drivingCommand.initialize();
      }

      @Override
      public void execute() {
        drivingCommand.execute();
      }

      @Override
      public void end(boolean interrupted) {
        fieldMap.removeTrajectoryFromField("Trajectory");
        drivingCommand.end(interrupted);
      }

      @Override
      public boolean isFinished() {
        return drivingCommand.isFinished();
      }
    };

    onTheFlyCommand.addRequirements(this);
    return onTheFlyCommand;

  }

  public CommandBase getOnTheFlyDriveCommand(List<Pose2d> positions, FieldMap fieldMap) {

    SwerveDrive swerveDrive = this;

    CommandBase onTheFlyCommand = new CommandBase() {
      private PPSwerveControllerCommand drivingCommand;

      @Override
      public void initialize() {
        Pose2d initialPose = swerveDrive.getSwerveDrivePosition();
        // if (DriverStation.getAlliance() == Alliance.Red) {
        // initialPose = new Pose2d(
        // initialPose.getX(),
        // 8.02 - initialPose.getY(),
        // new Rotation2d().minus(initialPose.getRotation()));
        // }

        PathPoint initialPoint = PathPoint.fromCurrentHolonomicState(
            initialPose,
            swerveDrive.getSwerveDriveChassisSpeed());

        Pose2d goalPose = getClosestPosition(swerveDrive, positions);

        PathPoint goalPoint = PathPoint.fromCurrentHolonomicState(goalPose, new ChassisSpeeds());

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(3, 3),
            initialPoint,
            goalPoint);

        // trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory,
        // DriverStation.getAlliance());

        drivingCommand = new PPSwerveControllerCommand(
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

        fieldMap.addTrajectoryToField("Trajectory", poses);

        drivingCommand.initialize();
      }

      @Override
      public void execute() {
        drivingCommand.execute();
      }

      @Override
      public void end(boolean interrupted) {
        fieldMap.removeTrajectoryFromField("Trajectory");
        drivingCommand.end(interrupted);
      }

      @Override
      public boolean isFinished() {
        return drivingCommand.isFinished();
      }
    };

    onTheFlyCommand.addRequirements(this);
    return onTheFlyCommand;

  }

  public CommandBase getOnRampCommand() {

    CommandBase wait = Commands.waitSeconds(1.0);

    CommandBase onRamp = new CommandBase() {
      @Override
      public void execute() {
        setSwerveDriveChassisSpeed(new ChassisSpeeds(4 / 1.4 * Math.sin(Math.toRadians(15)), 0, 0));
      }

      @Override
          public boolean isFinished() {
              double degrees = gyroscope.getPitch().getDegrees();
              if(degrees > 180){
                return (360 - degrees) >= 10; 
              }
              else{
                return degrees >= 10;
              }
          }
    };

    return onRamp;

  }

  public CommandBase getBalanceCommand() {
    CommandBase balance = new CommandBase() {

      @Override
      public void initialize() {
        SmartDashboard.putBoolean("Balancing", true);
      }

      @Override
      public void execute() {
        SmartDashboard.putBoolean("Balancing", false);
        Rotation2d pitchAngle = gyroscope.getPitch();
        Rotation2d pitchDistanceFrom0 = pitchAngle.minus(new Rotation2d());
        double pitchDistanceFrom0Radians = pitchDistanceFrom0.getRadians();

        double vxMetersPerSecond = -Constants.Robot.Drive.Modules.maxModuleSpeedMPS
            * Math.sin(pitchDistanceFrom0Radians) / 1.3;

        vxMetersPerSecond = MathUtil.applyDeadband(vxMetersPerSecond, 0.10);
        setSwerveDriveChassisSpeed(new ChassisSpeeds(vxMetersPerSecond, 0, 0));
      }

      @Override
      public void end(boolean interrupted) {
        stopDrive();
      }

      @Override
      public boolean isFinished() {
        // // Rotation2d pitchAngle = gyroscope.getPitch();
        // // Rotation2d pitchDistanceFrom0 = pitchAngle.minus(new Rotation2d());
        // // double pitchDistanceFrom0Value =
        // Math.abs(pitchDistanceFrom0.getDegrees());

        // return pitchDistanceFrom0Value < 2.5;
        return false;
      }
    };

    balance.addRequirements(this);
    return balance;
  }

  public CommandBase setTelemetryFromCameraCommand() {
    return Commands.runOnce(
        () -> setTelemetryFromCamera(),
        this);
  }

  public CommandBase setTelemetryFromSafeKnownPosition(Pose2d safeKnownPosition) {
    return Commands.runOnce(
        () -> resetSwerveDrivePosition(safeKnownPosition),
        this);
  }

  public CommandBase toggleAllowVisionMeasurementCommand() {
    return Commands.runOnce(
        () -> toggleAllowVisionMeasurements(),
        this);
  }

  public static Pose2d getClosestPosition(SwerveDrive swerveDrive, List<Pose2d> positions) {

    Pose2d currentPosition = swerveDrive.getSwerveDrivePosition();

    Pose2d closestPosition = positions.get(0);

    if (DriverStation.getAlliance() == Alliance.Red) {
      closestPosition = new Pose2d(closestPosition.getX(), 8.02 - closestPosition.getY(),
          closestPosition.getRotation());
    }
    Translation2d closestPositionTranslation = closestPosition.getTranslation();
    Translation2d currentPositionTranslation = currentPosition.getTranslation();

    double currentMinimumDistance = closestPositionTranslation.getDistance(currentPositionTranslation);

    for (int i = 1; i < positions.size(); i++) {
      Pose2d position = positions.get(i);
      if (DriverStation.getAlliance() == Alliance.Red) {
        position = new Pose2d(position.getX(), 8.02 - position.getY(), position.getRotation());
      }
      Translation2d positionTranslation = position.getTranslation();
      double distance = positionTranslation.getDistance(currentPositionTranslation);
      if (distance < currentMinimumDistance) {
        closestPosition = position;
        currentMinimumDistance = distance;
      }

    }

    return closestPosition;

  }

  public double getGyroPitch() {
    return gyroscope.getPitch().getDegrees();
  }

}
