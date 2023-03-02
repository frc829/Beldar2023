// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.telemetry.FieldMap;
import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class DriveAutoRoutines {

    public static CommandBase create(SwerveDrive swerveDrive, Pose2d position, FieldMap fieldMap) {

        return swerveDrive.getOnTheFlyDriveCommand(
                position,
                fieldMap);
    }

}
