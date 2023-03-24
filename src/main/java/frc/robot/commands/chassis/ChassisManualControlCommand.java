// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.controls.ManualChassisSpeedControl;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Telemetry;

public class ChassisManualControlCommand extends CommandBase {
  protected final SwerveDrive swerveDrive;
  protected final Telemetry telemetry;
  protected final ManualChassisSpeedControl manualChassisSpeedControl;

  public ChassisManualControlCommand(SwerveDrive swerveDrive, Telemetry telemetry) {
    this.swerveDrive = swerveDrive;
    this.telemetry = telemetry;
    this.manualChassisSpeedControl = swerveDrive.getManualChassisSpeedControl();
    addRequirements(swerveDrive, telemetry);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d robotAngle = telemetry.getSwerveDrivePosition().getRotation();
    ChassisSpeeds robotChassisSpeeds = manualChassisSpeedControl.getRobotCentricSpeeds(robotAngle);
    swerveDrive.setSwerveDriveChassisSpeed(robotChassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
      swerveDrive.stopDrive();
  }
}
