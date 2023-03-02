// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.mechanismsAdvanced;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.framework.mechanisms.LinearMech;
import frc.robot.framework.mechanisms.RotationMech;

public class SwerveModule {

  private final RotationMech steeringMechanism;
  private final LinearMech driveMechanism;

  public SwerveModule(
      RotationMech steeringMechanism,
      LinearMech driveMechanism) {
    this.driveMechanism = driveMechanism;
    this.steeringMechanism = steeringMechanism;
  }

  public SwerveModulePosition getSwerveModulePosition() {
    Rotation2d angle = steeringMechanism.getPositionRotations();
    double angleRotations = angle.getRotations();
    angleRotations %= 1.0;
    angleRotations = angleRotations < 0.0 ? angleRotations + 1.0 : angleRotations;
    angle = Rotation2d.fromRotations(angleRotations);
    double drivepos = driveMechanism.getPositionMeters();
    return new SwerveModulePosition(drivepos, angle);
  }

  public SwerveModuleState getSwerveModuleState() {
    Rotation2d angle = steeringMechanism.getPositionRotations();
    double angleRotations = angle.getRotations();
    angleRotations %= 1.0;
    angleRotations = angleRotations < 0.0 ? angleRotations + 1.0 : angleRotations;
    angle = Rotation2d.fromRotations(angleRotations);
    double drivespeed = driveMechanism.getSpeedMetersPerSecond();
    return new SwerveModuleState(drivespeed, angle);
  }

  public void setSwerveModuleState(Rotation2d steeringMechanismVelocityRPS, double driveMechanismVelocityMPS) {
    this.steeringMechanism.setVelocityRotationsPerSecond(steeringMechanismVelocityRPS);
    driveMechanism.setMechanismSpeedMetersPerSecond(driveMechanismVelocityMPS);
  }

  public void setSwerveModuleSteeringEncoder() {
    this.steeringMechanism.setEncoderPositionFromSensor();
  }

  public void stop() {
    this.driveMechanism.stop();
    this.steeringMechanism.stop();
  }
}
