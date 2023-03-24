// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Telemetry;

public class ChassisRobotCentricDriveForever extends ChassisManualControlCommand {




  public ChassisRobotCentricDriveForever(
    SwerveDrive swerveDrive, 
    Telemetry telemetry,
    double forwardSpeedMetersPerSecond,
    double strafeSpeedMetersPerSecond,
    double rotationalSpeedDegreesPerSecond) {
    super(swerveDrive, telemetry);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d robotAngle = telemetry.getSwerveDrivePosition().getRotation();
    ChassisSpeeds robotChassisSpeeds = manualChassisSpeedControl.getRobotCentricSpeeds(robotAngle);
    swerveDrive.setSwerveDriveChassisSpeed(robotChassisSpeeds);
  }
}
