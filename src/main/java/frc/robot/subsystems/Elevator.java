// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.framework.controls.ManualSpeedControl;
import frc.robot.framework.mechanisms.LinearMech;

public class Elevator extends SubsystemBase {

  private final LinearMech elevatorMech;
  private final ManualSpeedControl manualSpeedControl;
  private final PIDController elevatorPIDController;
  private final double elevatorMinimumPositionMeters;
  private final double elevatorMaximumPositionMeters;
  private final MechanismLigament2d elevatorMech2d;
  private final DecimalFormat decimalFormat;

  public Elevator(
      LinearMech elevatorMech,
      ManualSpeedControl manualSpeedControl,
      PIDController elevatorPIDController,
      double elevatorMinimumPositionMeters,
      double elevatorMaximumPositionMeters,
      MechanismLigament2d elevatorMech2d,
      DecimalFormat decimalFormat) {
    this.elevatorMech = elevatorMech;
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
        decimalFormat.format(getPosition()));
    SmartDashboard.putString(
        "Elevator Position From Sensor (m)",
        decimalFormat.format(getPositionFromSensor()));
    SmartDashboard.putString(
        "Elevator Speed From Motor(mps)",
        decimalFormat.format(getVelocity()));

    this.elevatorMech2d.setLength(getPosition());
  }

  private double getPosition() {
    return elevatorMech.getPositionMeters();
  }

  private double getVelocity() {
    return elevatorMech.getSpeedMetersPerSecond();
  }

  private double getPositionFromSensor() {
    return elevatorMech.getLinearPositionFromSensor();
  }

  public CommandBase createHoldCommand() {

    Runnable initPIDController = new Runnable() {

      @Override
      public void run() {
        elevatorPIDController.setSetpoint(getPosition());
      }

    };

    Runnable control = new Runnable() {

      @Override
      public void run() {
        double velocityMetersPerSecond = elevatorPIDController.calculate(getPosition());
        elevatorMech.setMechanismSpeedMetersPerSecond(velocityMetersPerSecond);
        velocityMetersPerSecond = elevatorPIDController.atSetpoint() ? 0 : velocityMetersPerSecond;
        elevatorMech.setMechanismSpeedMetersPerSecond(velocityMetersPerSecond);
      }
    };

    CommandBase initializePIDControllerCommand = Commands.runOnce(initPIDController, this);
    CommandBase runElbowToPosition = Commands.run(control, this);

    return Commands.sequence(initializePIDControllerCommand, runElbowToPosition);
  }

  public CommandBase createControlCommand() {
    Runnable control = new Runnable() {

      @Override
      public void run() {
        double velocityMPS = manualSpeedControl.getManualSpeed();
        velocityMPS = getPosition() <= elevatorMinimumPositionMeters && velocityMPS < 0.0 ? 0.0 : velocityMPS;
        velocityMPS = getPosition() >= elevatorMaximumPositionMeters && velocityMPS > 0.0 ? 0.0 : velocityMPS;
        elevatorMech.setMechanismSpeedMetersPerSecond(velocityMPS);
      }
    };

    return Commands.run(control, this);
  }

  public PIDCommand createPickupControlCommand(double positionMeters) {
    PIDCommand pidCommand = new PIDCommand(
        elevatorPIDController,
        this::getPosition,
        positionMeters,
        this.elevatorMech::setMechanismSpeedMetersPerSecond,
        this);
    return pidCommand;
  }

  public PIDCommand createControlCommand(double positionMeters) {

    PIDCommand pidCommand = new PIDCommand(
        elevatorPIDController,
        this::getPosition,
        positionMeters,
        this.elevatorMech::setMechanismSpeedMetersPerSecond,
        this) {
      @Override
      public boolean isFinished() {
        return elevatorPIDController.atSetpoint();
      }
    };

    return pidCommand;

  }

  public CommandBase createStopCommand() {
    Runnable control = new Runnable() {

      @Override
      public void run() {
        elevatorMech.stop();
      }
    };

    return Commands.runOnce(control, this);
  }
}
