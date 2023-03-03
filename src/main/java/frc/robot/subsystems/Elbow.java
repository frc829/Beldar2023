// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.framework.controls.ManualSpeedControl;
import frc.robot.framework.mechanisms.MotorReduction;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.sensors.AngularPositionSensor;

public class Elbow extends SubsystemBase {

  private final Motor motor;
  private final MotorReduction motorReduction;
  private final AngularPositionSensor angularPositionSensor;
  private final ManualSpeedControl manualSpeedControl;
  private final PIDController elbowPIDController;
  private final double elbowMinimumPositionDegrees;
  private final double elbowMaximumPositionDegrees;
  private final MechanismLigament2d elbowMech2d;
  private final DecimalFormat decimalFormat;

  public Elbow(
      Motor motor,
      MotorReduction motorReduction,
      AngularPositionSensor angularPositionSensor,
      ManualSpeedControl manualSpeedControl,
      PIDController elbowPIDController,
      double elbowMinimumPositionDegrees,
      double elbowMaximumPositionDegrees,
      MechanismLigament2d elbowMech2d,
      DecimalFormat decimalFormat) {

    this.motor = motor;
    this.motorReduction = motorReduction;
    this.angularPositionSensor = angularPositionSensor;
    this.manualSpeedControl = manualSpeedControl;
    this.elbowPIDController = elbowPIDController;
    this.elbowMinimumPositionDegrees = elbowMinimumPositionDegrees;
    this.elbowMaximumPositionDegrees = elbowMaximumPositionDegrees;
    this.elbowMech2d = elbowMech2d;
    this.decimalFormat = decimalFormat;

    setMotorEncoderFromSensor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString(
        "Elbow Position From Motor (deg)",
        decimalFormat.format(getPosition() * 360.0));
    SmartDashboard.putString(
        "Elbow Position From Sensor (deg)",
        decimalFormat.format(getPositionFromSensor().getDegrees()));
    SmartDashboard.putString(
        "Elbow Speed From Motor(degps)",
        decimalFormat.format(getVelocity() * 360.0));

    this.elbowMech2d.setAngle(getPosition());
  }

  // Suppliers
  private double getPosition() {
    Rotation2d motorRotations = motor.getPosition();
    Rotation2d motorRotationsReduced = motorReduction.reduceMotorRotations(motorRotations);
    return motorRotationsReduced.getRotations();
  }

  private double getVelocity() {
    Rotation2d motorRotationsPerSecond = motor.getVelocity();
    Rotation2d motorRotationsPerSecondReduced = motorReduction.reduceMotorRotations(motorRotationsPerSecond);
    return motorRotationsPerSecondReduced.getRotations();
  }

  private Rotation2d getPositionFromSensor() {
    return angularPositionSensor.getAbsoluteAngle();
  }

  // Consumers
  private void setVelocity(double velocityRotationsPerSecond) {
    velocityRotationsPerSecond = velocityRotationsPerSecond >= 0
        && getPosition() * 360.0 > elbowMinimumPositionDegrees ? 0
            : velocityRotationsPerSecond;
    velocityRotationsPerSecond = velocityRotationsPerSecond <= 0
        && getPosition() * 360.0 < elbowMaximumPositionDegrees ? 0
            : velocityRotationsPerSecond;
    Rotation2d mechRotationsPerSecond = Rotation2d.fromRotations(velocityRotationsPerSecond);
    Rotation2d motorRotations = motorReduction.expandMechanismRotations(mechRotationsPerSecond);
    motor.setVelocity(motorRotations);
  }

  private void setMotorEncoderFromSensor() {
    Rotation2d positionFromSensor = getPositionFromSensor();
    Rotation2d motorRotations = motorReduction.expandMechanismRotations(positionFromSensor);
    motor.setPosition(motorRotations);
  }

  // Commands
  public PIDCommand createHoldCommand() {

    PIDCommand pidHoldCommand = new PIDCommand(
        elbowPIDController,
        this::getPosition,
        this::getPosition,
        this::setVelocity,
        this);

    return pidHoldCommand;
  }

  public CommandBase createManualControlCommand() {
    return new CommandBase() {
      @Override
      public void execute() {
          double velocityMetersPerSecond = manualSpeedControl.getManualSpeed();
          setVelocity(velocityMetersPerSecond);
      }
    };
  }

  public PIDCommand createSetPositionCommand(double positionRotations) {

    PIDCommand setPositionCommand = new PIDCommand(
      elbowPIDController, 
      this::getPosition, 
      positionRotations, 
      this::setVelocity, 
      this){
        @Override
        public boolean isFinished() {
            return elbowPIDController.atSetpoint();
        }
      };

      return setPositionCommand;
  }

  public CommandBase createStopCommand() {
    return Commands.runOnce(motor::stop, this);
  }
}
