// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.framework.imus.Gyroscope;
import frc.robot.framework.imus.NavXGyroscopeFactory;
import frc.robot.framework.mechanismsAdvanced.SwerveModule;
import frc.robot.framework.telemetry.FieldMap;
import frc.robot.framework.telemetry.FieldTelemetry;
import frc.robot.framework.telemetry.SwervePoseEstimatorFactory;
import frc.robot.framework.vision.TrackingCamera;

/** Add your docs here. */
public class Telemetry extends SubsystemBase {

        private final Gyroscope gyroscope;
        private final FieldTelemetry fieldTelemetry;
        private final TrackingCamera trackingCamera;
        private final FieldMap fieldMap;

        public Telemetry(
                        Pose2d initialPosition,
                        FieldMap fieldMap,
                        SwerveDriveKinematics swerveDriveKinematics,
                        SwerveModule... swerveModules) {

                SwerveDrivePoseEstimator swerveDrivePoseEstimator = SwervePoseEstimatorFactory.create(
                                swerveDriveKinematics,
                                new Rotation2d(),
                                initialPosition,
                                swerveModules);

                this.fieldTelemetry = FieldTelemetry.create(
                                swerveDrivePoseEstimator,
                                swerveModules);

                AHRS navXMXP2 = NavXGyroscopeFactory.create(
                                Constants.Robot.Drive.Gyroscope.serial_port_id,
                                swerveDriveKinematics,
                                swerveModules);

                this.gyroscope = Gyroscope.create(navXMXP2);

                this.trackingCamera = TrackingCamera.createFromLimeLight("limelight");

                this.fieldMap = fieldMap;
        }

        @Override
        public void periodic() {
                this.fieldTelemetry.updateCurrentPosition(this.gyroscope.getYaw());
                double[] currentPoseFromCamera = this.trackingCamera.getFieldPosition(DriverStation.getAlliance());
                this.fieldTelemetry.addVisionMeasurement(currentPoseFromCamera);
                fieldMap.updateField(this.fieldTelemetry.getCurrentPosition());

                // SmartDashboard.putBoolean("Gyro Connected", this.gyroscope.isConnected());
                SmartDashboard.putNumber("PoseFromCameraX", currentPoseFromCamera[0]);
                SmartDashboard.putNumber("PoseFromCameraY", currentPoseFromCamera[1]);
                SmartDashboard.putNumber("PoseFromCameraZ", currentPoseFromCamera[2]);
                SmartDashboard.putNumber("PoseFromCameraRoll", currentPoseFromCamera[3]);
                SmartDashboard.putNumber("PoseFromCameraPitch", currentPoseFromCamera[4]);
                SmartDashboard.putNumber("PoseFromCameraYaw", currentPoseFromCamera[5]);

                // SmartDashboard.putNumber("Gyro Pitch",
                // this.gyroscope.getPitch().getDegrees());
                // SmartDashboard.putNumber("Gyro Roll", this.gyroscope.getRoll().getDegrees());
                // SmartDashboard.putNumber("Gyro Yaw", this.gyroscope.getYaw().getDegrees());

                // SmartDashboard.putNumber("GyroRoll",
                // (this.gyroscope.getRoll().getDegrees()));
                // SmartDashboard.putNumber("GyroPitch",
                // (this.gyroscope.getPitch().getDegrees()));
                // SmartDashboard.putNumber("GyroYaw", (this.gyroscope.getYaw().getDegrees()));

                this.fieldTelemetry.updateCurrentPosition(gyroscope.getYaw());
        }

        // Functions
        public Pose2d getClosestPosition(List<Pose2d> positions) {

                Pose2d currentPosition = this.getSwerveDrivePosition();

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

        // Suppliers
        public TrackingCamera getTrackingCamera() {
                return this.trackingCamera;
        }

        public Pose2d getSwerveDrivePosition() {
                return this.fieldTelemetry.getCurrentPosition();
        }

        public double getGyroPitch() {
                return gyroscope.getPitch().getDegrees();
        }

        // Runnables

        // Consumers
        public void resetSwerveDrivePosition(Pose2d newPose) {
                this.fieldTelemetry.resetCurrentPosition(newPose, this.gyroscope.getYaw());
        }

        public void setTelemetryFromCamera() {

                double[] currentPoseFromCamera = this.trackingCamera.getFieldPosition(DriverStation.getAlliance());
                Rotation2d yawFromCamera = Rotation2d.fromDegrees(currentPoseFromCamera[5]);
                Pose2d cameraPose = new Pose2d(currentPoseFromCamera[0], currentPoseFromCamera[1], yawFromCamera);
                this.fieldTelemetry.resetCurrentPosition(cameraPose, gyroscope.getYaw());
        }

        // Commands
        public CommandBase setTelemetryFromSafeKnownPosition(Pose2d safeKnownPosition) {
                return Commands.runOnce(
                                () -> resetSwerveDrivePosition(safeKnownPosition),
                                this);
        }

        public CommandBase setTelemetryFromCameraCommand() {
                return Commands.runOnce(
                                () -> setTelemetryFromCamera(),
                                this);
        }

}
