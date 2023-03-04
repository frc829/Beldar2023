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
import frc.robot.framework.mechanisms.RotationMech;

public class Elbow extends SubsystemBase {

  private final RotationMech elbowMech;
  private final ManualSpeedControl manualSpeedControl;
  private final PIDController elbowPIDController;
  private final double elbowMinimumPositionDegrees;
  private final double elbowMaximumPositionDegrees;
  private final MechanismLigament2d elbowMech2d;
  private final DecimalFormat decimalFormat;

  public Elbow(
      RotationMech elbowMech,
      ManualSpeedControl manualSpeedControl,
      PIDController elbowPIDController,
      double elbowMinimumPositionDegrees,
      double elbowMaximumPositionDegrees,
      MechanismLigament2d elbowMech2d,
      DecimalFormat decimalFormat) {
    this.elbowMech = elbowMech;
    this.manualSpeedControl = manualSpeedControl;
    this.elbowPIDController = elbowPIDController;
    this.elbowMinimumPositionDegrees = elbowMinimumPositionDegrees;
    this.elbowMaximumPositionDegrees = elbowMaximumPositionDegrees;
    this.elbowMech2d = elbowMech2d;
    this.decimalFormat = decimalFormat;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString(
        "Elbow Position From Motor (deg)",
        decimalFormat.format(getPosition().getDegrees()));
    SmartDashboard.putString(
        "Elbow Position From Sensor (deg)",
        decimalFormat.format(getPositionFromSensor().getDegrees()));
    SmartDashboard.putString(
        "Elbow Speed From Motor(degps)",
        decimalFormat.format(getVelocity().getDegrees()));
    this.elbowMech2d.setAngle(getPosition().getDegrees());
  }

  private Rotation2d getPosition() {
    return elbowMech.getPositionRotations();
  }

  private double getPositionRotations() {
    return getPosition().getRotations();
  }

  private Rotation2d getVelocity() {
    return elbowMech.getVelocityRotationPerSecond();
  }

  private Rotation2d getPositionFromSensor() {
    return elbowMech.getAngularPostionFromSensor();
  }

  private void setVelocityRotationsPerSecond(double rotationsPerSecond){
    Rotation2d rps = Rotation2d.fromRotations(rotationsPerSecond);
    this.elbowMech.setVelocityRotationsPerSecond(rps);
  }

  public CommandBase createHoldCommand() {

    Runnable initPIDController = new Runnable() {

      @Override
      public void run() {
        elbowPIDController.setSetpoint(getPosition().getRotations());
      }

    };

    Runnable control = new Runnable() {

      @Override
      public void run() {
        double velocityRotationsPerSecond = elbowPIDController.calculate(getPosition().getRotations());
        Rotation2d velocityRotation2dPerSecond = Rotation2d.fromRotations(velocityRotationsPerSecond);
        elbowMech.setVelocityRotationsPerSecond(velocityRotation2dPerSecond);
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
        double velocityRPS = manualSpeedControl.getManualSpeed();
        velocityRPS = getPosition().getDegrees() <= elbowMinimumPositionDegrees && velocityRPS < 0.0 ? 0.0
            : velocityRPS;
        velocityRPS = getPosition().getDegrees() >= elbowMaximumPositionDegrees && velocityRPS > 0.0 ? 0.0
            : velocityRPS;
        Rotation2d velocityRotationsPerSecond = Rotation2d.fromRotations(velocityRPS);
        elbowMech.setVelocityRotationsPerSecond(velocityRotationsPerSecond);
      }
    };

    return Commands.run(control, this);
  }

  public CommandBase createControlCommand(double positionDegrees) {

    

    PIDCommand pidCommand = new PIDCommand(
        elbowPIDController,
        this::getPositionRotations,
        positionDegrees / 360.0,
        this::setVelocityRotationsPerSecond,
        this) {
      @Override
      public boolean isFinished() {
        return elbowPIDController.atSetpoint();
      }
    };

    return pidCommand;

  }

  public CommandBase createStopCommand() {
    Runnable control = new Runnable() {

      @Override
      public void run() {
        elbowMech.stop();
      }
    };

    return Commands.runOnce(control, this);
  }
}
