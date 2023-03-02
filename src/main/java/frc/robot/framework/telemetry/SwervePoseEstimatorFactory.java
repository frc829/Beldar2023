// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.telemetry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.framework.mechanismsAdvanced.SwerveModule;

/** Add your docs here. */
public class SwervePoseEstimatorFactory {

    public static SwerveDrivePoseEstimator create(
            SwerveDriveKinematics swerveDriveKinematics,
            Rotation2d gyroAngle,
            Pose2d initialPoseMeters,
            SwerveModule... swerveModules) {

        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModulePositions[i] = swerveModules[i].getSwerveModulePosition();
        }

        return new SwerveDrivePoseEstimator(
                swerveDriveKinematics,
                gyroAngle,
                swerveModulePositions,
                initialPoseMeters);

    }
}
