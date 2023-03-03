// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.mechanismsAdvanced;

import edu.wpi.first.math.geometry.Rotation2d;
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

    setSteeringMechEncoderFromSensor();
  }


  //Suppliers
  public Rotation2d getSteeringMechPosition(){
    Rotation2d steeringMotorPosition = steeringMotor.getPosition();
    Rotation2d steeringMechPosition = steeringMotorReduction.reduceMotorRotations(steeringMotorPosition);
    double steeringMechPositionRotations = steeringMechPosition.getRotations();
    steeringMechPositionRotations %= 1.0;
    steeringMechPositionRotations = steeringMechPositionRotations < 0 ? steeringMechPositionRotations + 1.0 : steeringMechPositionRotations;
    steeringMechPosition = Rotation2d.fromRotations(steeringMechPositionRotations);
    return steeringMechPosition;
  }

  public double getDriveMechPosition(){
    Rotation2d driveMotorPosition = driveMotor.getPosition();
    Rotation2d driveMechPosition = driveMotorReduction.reduceMotorRotations(driveMotorPosition);
    double drivePosition = driveWheel.transformRotationToLinear(driveMechPosition);
    return drivePosition;
  }

  public double getDriveMechSpeed(){
    Rotation2d driveMotorVelocity = driveMotor.getVelocity();
    Rotation2d driveMechVelocity = driveMotorReduction.reduceMotorRotations(driveMotorVelocity);
    double driveVelocity = driveWheel.transformRotationToLinear(driveMechVelocity);
    return driveVelocity;
  }

  public Rotation2d getSteeringPositionFromSensor(){
    return steeringSensor.getAbsoluteAngle();
  }

  //Runnables
  
  public void setSteeringMechEncoderFromSensor(){
    Rotation2d positionFromSensor = getSteeringPositionFromSensor();
    Rotation2d motorRotations = steeringMotorReduction.expandMechanismRotations(positionFromSensor);
    steeringMotor.setPosition(motorRotations);
  }
  
  public void stop(){
    steeringMotor.stop();
    driveMotor.stop();
  }

  //Consumers
  public void setSteeringMechSpeed(Rotation2d velocityPerSecond){
    Rotation2d motorVelocityPerSecond = steeringMotorReduction.expandMechanismRotations(velocityPerSecond);
    steeringMotor.setVelocity(motorVelocityPerSecond);
  }

  public void setDriveMechSpeed(double velocityMetersPerSecond){
    Rotation2d driveMechVelocity = driveWheel.transformLinearToRotation(velocityMetersPerSecond);
    Rotation2d driveMotorVelocity = driveMotorReduction.expandMechanismRotations(driveMechVelocity);
    driveMotor.setVelocity(driveMotorVelocity);
  }


}
