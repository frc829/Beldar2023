// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.telemetry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.framework.mechanismsAdvanced.SwerveModule;

/** Add your docs here. */
public interface FieldTelemetry {

    public Pose2d getCurrentPosition();

    public void updateCurrentPosition(Rotation2d gyroAngle);

    public void addVisionMeasurement(double[] poseFromVision);

    public void resetCurrentPosition(
            Pose2d newPose,
            Rotation2d gyroScopeAngle);

    public static FieldTelemetry create(
            SwerveDrivePoseEstimator swerveDrivePoseEstimator,
            SwerveModule... swerveModules) {

        return new FieldTelemetry() {

            @Override
            public Pose2d getCurrentPosition() {
                return swerveDrivePoseEstimator.getEstimatedPosition();
            }

            @Override
            public void updateCurrentPosition(Rotation2d gyroAngle) {

                SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];

                for (int i = 0; i < swerveModules.length; i++) {
                    swerveModulePositions[i] = swerveModules[i].getSwerveModulePosition();
                }

                swerveDrivePoseEstimator.updateWithTime(
                        Timer.getFPGATimestamp(),
                        gyroAngle,
                        swerveModulePositions);
            }

            @Override
            public void addVisionMeasurement(double[] poseFromVision) {

                Translation2d poseFromVisionTranslation = new Translation2d(
                        poseFromVision[0],
                        poseFromVision[1]);
                Translation2d currentPoseFromEstimatorTranslation = getCurrentPosition().getTranslation();
                double poseDifferencesNorm = poseFromVisionTranslation.getDistance(currentPoseFromEstimatorTranslation);

                Rotation2d currentYaw = getCurrentPosition().getRotation();
                Rotation2d poseFromVisionyaw = Rotation2d.fromDegrees(poseFromVision[5]);
                Rotation2d yawDifference = currentYaw.minus(poseFromVisionyaw);

                double yawDifferenceDegrees = yawDifference.getDegrees();
                double yawDifferenceDegreesMag = Math.abs(yawDifferenceDegrees);

                // SmartDashboard.putNumber("YAWFROMTELE", poseFromVision[5]);

                if (poseFromVision[0] != 0) {
                    if (poseDifferencesNorm <= 1.0 && yawDifferenceDegreesMag <= 2.0) {
                        swerveDrivePoseEstimator.addVisionMeasurement(
                                new Pose2d(poseFromVision[0], poseFromVision[1],
                                        Rotation2d.fromDegrees(poseFromVision[5])),
                                Timer.getFPGATimestamp());
                    } else if (poseDifferencesNorm <= 1.0) {
                        Pose2d visionPose = new Pose2d(
                                poseFromVision[0],
                                poseFromVision[1],
                                getCurrentPosition().getRotation());
                        swerveDrivePoseEstimator.addVisionMeasurement(
                                visionPose,
                                Timer.getFPGATimestamp());
                    } else if (yawDifferenceDegreesMag <= 2.0) {
                        Pose2d visionPose = new Pose2d(
                                getCurrentPosition().getX(),
                                getCurrentPosition().getY(),
                                Rotation2d.fromDegrees(poseFromVision[5]));
                        swerveDrivePoseEstimator.addVisionMeasurement(
                                visionPose,
                                Timer.getFPGATimestamp());
                    }
                }

            }

            @Override
            public void resetCurrentPosition(Pose2d newPose, Rotation2d gyroScopeAngle) {

                SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];

                for (int i = 0; i < swerveModules.length; i++) {
                    swerveModulePositions[i] = swerveModules[i].getSwerveModulePosition();
                }

                swerveDrivePoseEstimator.resetPosition(
                        gyroScopeAngle,
                        swerveModulePositions,
                        newPose);

            }
        };
    }
}
