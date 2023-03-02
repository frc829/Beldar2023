// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.kinematics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class KinematicsFactory {

    public static SwerveDriveKinematics createSwerveKinematics(
            Translation2d... translation2ds) {
        return new SwerveDriveKinematics(translation2ds);
    }

}
