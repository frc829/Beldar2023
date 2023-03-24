// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Telemetry;

public class ChassisDefaultCommand extends CommandBase {
  protected final SwerveDrive swerveDrive;
  protected final Telemetry telemetry;

  public ChassisDefaultCommand(SwerveDrive swerveDrive, Telemetry telemetry) {
    this.swerveDrive = swerveDrive;
    this.telemetry = telemetry;
    addRequirements(swerveDrive, telemetry);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
