// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  // Suppliers
  private double getPosition() {
    return elevatorMech.getPositionMeters();
  }

  private double getVelocity() {
    return elevatorMech.getSpeedMetersPerSecond();
  }

  private double getPositionFromSensor() {
    return elevatorMech.getLinearPositionFromSensor();
  }

  // Runnable
  private void stop() {
    this.elevatorMech.stop();
  }

  // Consumers
  private void setVelocity(double speedMetersPerSecond) {
    this.elevatorMech.setMechanismSpeedMetersPerSecond(speedMetersPerSecond);
  }

  public CommandBase createHoldCommand() {

    PIDCommand pidCommand = new PIDCommand(
        elevatorPIDController,
        this::getPosition,
        getPosition(),
        this::setVelocity,
        this) {
      @Override
      public void initialize() {
        this.m_setpoint = () -> getPosition();
        SmartDashboard.putString("Elevator Command Current", "Hold at " + this.m_setpoint.getAsDouble() + " m");
        super.initialize();
      }
    };

    return pidCommand;
  }

  public CommandBase createControlCommand() {

    CommandBase manualControlCommand = new CommandBase() {

      @Override
      public void initialize() {
        SmartDashboard.putString("Elevator Command Current", "Manual Control");
      }

      @Override
      public void execute() {
        double manualSpeed = manualSpeedControl.getManualSpeed();
        setVelocity(manualSpeed);
      }

    };

    manualControlCommand.addRequirements(this);
    return manualControlCommand;

  }

  public PIDCommand createControlCommand(double positionMeters) {

    PIDCommand pidCommand = new PIDCommand(
        elevatorPIDController,
        this::getPosition,
        positionMeters,
        this::setVelocity,
        this) {

      @Override
      public void initialize() {
        SmartDashboard.putString("Elevator Command Current", "Go To " + positionMeters + " meters");
        super.initialize();
      }

      @Override
      public boolean isFinished() {
        return elevatorPIDController.atSetpoint();
      }
    };

    return pidCommand;

  }

  public CommandBase createStopCommand() {

    CommandBase stopCommand = new CommandBase() {
      @Override
      public void initialize() {
        SmartDashboard.putString("Elevator Command Current", "Stop");
      }

      @Override
      public void execute() {
        stop();
      }

      @Override
      public boolean isFinished() {
        return true;
      }

    };

    stopCommand.addRequirements(this);

    return stopCommand;
  }
}
