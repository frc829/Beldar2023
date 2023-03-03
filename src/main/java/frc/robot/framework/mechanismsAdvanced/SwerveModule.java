// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.mechanismsAdvanced;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.framework.mechanisms.MotorReduction;
import frc.robot.framework.mechanisms.Wheel;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.sensors.AngularPositionSensor;

public class SwerveModule {

  private final Motor steeringMotor;
  private final Motor driveMotor;
  private final MotorReduction steeringMotorReduction;
  private final MotorReduction driveMotorReduction;
  private final Wheel driveWheel;
  private final AngularPositionSensor steeringSensor;

  public SwerveModule(
      Motor steeringMotor,
      Motor driveMotor,
      MotorReduction steeringMotorReduction,
      MotorReduction driveMotorReduction,
      Wheel driveWheel,
      AngularPositionSensor steeringSensor) {
    this.steeringMotor = steeringMotor;
    this.driveMotor = driveMotor;
    this.steeringMotorReduction = steeringMotorReduction;
    this.driveMotorReduction = driveMotorReduction;
    this.driveWheel = driveWheel;
    this.steeringSensor = steeringSensor;
  }


  //Suppliers
  public Rotation2d getSteeringMotorPosition(){
    Rotation2d steeringMotorPosition = steeringMotor.getPosition();
    Rotation2d steeringMechPosition = steeringMotorReduction.reduceMotorRotations(steeringMotorPosition);
    double steeringMechPositionRotations = steeringMechPosition.getRotations();
    steeringMechPositionRotations %= 1.0;
    steeringMechPositionRotations = steeringMechPositionRotations < 0 ? steeringMechPositionRotations + 1.0 : steeringMechPositionRotations;
    steeringMechPosition = Rotation2d.fromRotations(steeringMechPositionRotations);
    return steeringMechPosition;
  }

  public double getDriveMotorPosition(){

  }

  public double getDriveMotorSpeed(){

  }

  //Consumers


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
    this.steeringMotor.stop();
    this.driveMotor.stop();
  }
}
