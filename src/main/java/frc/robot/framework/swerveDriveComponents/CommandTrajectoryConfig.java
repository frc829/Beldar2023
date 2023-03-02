// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.swerveDriveComponents;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class CommandTrajectoryConfig {
    public final Command command;
    public final List<Pose2d> bluePoses;
    public final List<Pose2d> redPoses;

    public CommandTrajectoryConfig(
        Command command,
        List<Pose2d> bluePoses,
        List<Pose2d> redPoses
    ){
        this.command = command;
        this.bluePoses = bluePoses;
        this.redPoses = redPoses;
    }
}
