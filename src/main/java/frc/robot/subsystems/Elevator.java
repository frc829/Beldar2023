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
import frc.robot.framework.mechanisms.Wheel;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.sensors.LinearPositionSensor;
import frc.robot.types.PIDEndAtSetPointCommand;

public class Elevator extends SubsystemBase {

  private final Motor motor;
  private final MotorReduction motorReduction;
  private final Wheel chainSprocket;
  private final LinearPositionSensor linearPositionSensor;
  private final ManualSpeedControl manualSpeedControl;
  private final PIDController elevatorPIDController;
  private final double elevatorMinimumPositionMeters;
  private final double elevatorMaximumPositionMeters;
  private final MechanismLigament2d elevatorMech2d;
  private final DecimalFormat decimalFormat;

  public Elevator(
      Motor motor,
      MotorReduction motorReduction,
      Wheel chainSprocket,
      LinearPositionSensor linearPositionSensor,
      ManualSpeedControl manualSpeedControl,
      PIDController elevatorPIDController,
      double elevatorMinimumPositionMeters,
      double elevatorMaximumPositionMeters,
      MechanismLigament2d elevatorMech2d,
      DecimalFormat decimalFormat) {
    this.motor = motor;
    this.motorReduction = motorReduction;
    this.chainSprocket = chainSprocket;
    this.linearPositionSensor = linearPositionSensor;
    this.manualSpeedControl = manualSpeedControl;
    this.elevatorPIDController = elevatorPIDController;
    this.elevatorMinimumPositionMeters = elevatorMinimumPositionMeters;
    this.elevatorMaximumPositionMeters = elevatorMaximumPositionMeters;
    this.elevatorMech2d = elevatorMech2d;
    this.decimalFormat = decimalFormat;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString(
        "Elevator Position From Motor (m)",
        decimalFormat.format(getPositionMeters()));
    SmartDashboard.putString(
        "Elevator Position From Sensor (m)",
        decimalFormat.format(getPositionFromSensor()));
    SmartDashboard.putString(
        "Elevator Speed From Motor(mps)",
        decimalFormat.format(getVelocity()));

    this.elevatorMech2d.setLength(getPositionMeters());
  }

  // Suppliers
  private double getPositionMeters() {
    Rotation2d motorRotations = motor.motorEncoder.getPosition();
    Rotation2d motorRotationsReduced = motorReduction.reduceMotorRotations(motorRotations);
    double linearPositionMeters = chainSprocket.transformRotationToLinear(motorRotationsReduced);
    return linearPositionMeters;
  }

  private double getVelocity() {
    Rotation2d motorRotationsPerSecond = motor.motorEncoder.getVelocityPerSecond();
    Rotation2d motorRotationsPerSecondReduced = motorReduction.reduceMotorRotations(motorRotationsPerSecond);
    double linearVelocityMetersPerSecond = chainSprocket.transformRotationToLinear(motorRotationsPerSecondReduced);
    return linearVelocityMetersPerSecond;
  }

  private double getPositionFromSensor() {
    return linearPositionSensor.getPosition();
  }

  // Runnables
  private void setElevatorSpeedManual() {
    double velocityMetersPerSecond = manualSpeedControl.getManualSpeed();
    velocityMetersPerSecond = velocityMetersPerSecond >= 0 && getPositionMeters() > elevatorMaximumPositionMeters ? 0
        : velocityMetersPerSecond;
    velocityMetersPerSecond = velocityMetersPerSecond <= 0 && getPositionMeters() < elevatorMinimumPositionMeters ? 0
        : velocityMetersPerSecond;
    Rotation2d mechRotationsPerSecond = chainSprocket.transformLinearToRotation(velocityMetersPerSecond);
    Rotation2d motorRotations = motorReduction.expandMechanismRotations(mechRotationsPerSecond);
    motor.motorController.setVelocity(motorRotations);
  }

  // Consumers
  private void setVelocity(double velocityMetersPerSecond) {
    velocityMetersPerSecond = velocityMetersPerSecond >= 0 && getPositionMeters() > elevatorMaximumPositionMeters ? 0
        : velocityMetersPerSecond;
    velocityMetersPerSecond = velocityMetersPerSecond <= 0 && getPositionMeters() < elevatorMinimumPositionMeters ? 0
        : velocityMetersPerSecond;
    Rotation2d mechRotationsPerSecond = chainSprocket.transformLinearToRotation(velocityMetersPerSecond);
    Rotation2d motorRotations = motorReduction.expandMechanismRotations(mechRotationsPerSecond);
    motor.motorController.setVelocity(motorRotations);
  }

  // Commands
  public PIDCommand createHoldCommand() {

    PIDCommand pidHoldCommand = new PIDCommand(
        elevatorPIDController,
        this::getPositionMeters,
        getPositionMeters(),
        this::setVelocity,
        this);

    return pidHoldCommand;
  }

  public CommandBase createManualControlCommand() {
    return Commands.run(this::setElevatorSpeedManual, this);
  }

  public PIDEndAtSetPointCommand createSetPositionCommand(double positionMeters) {

    PIDEndAtSetPointCommand pidEndAtSetPointCommand = new PIDEndAtSetPointCommand(
        elevatorPIDController,
        this::getPositionMeters,
        positionMeters,
        this::setVelocity,
        this);

    return pidEndAtSetPointCommand;
  }

  public CommandBase createStopCommand() {
    return Commands.runOnce(motor.motorController::stop, this);
  }
}
