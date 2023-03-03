// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.framework.controls.ManualSpeedControl;
import frc.robot.framework.mechanismsAdvanced.DualRotationMechWithLimitSwitch;

public class Grabber extends SubsystemBase {

  private final DualRotationMechWithLimitSwitch grabberMech;
  private final ManualSpeedControl manualSpeedControl;
  private final DecimalFormat decimalFormat;

  public Grabber(
      DualRotationMechWithLimitSwitch grabberMech,
      ManualSpeedControl manualSpeedControl,
      DecimalFormat decimalFormat) {
    this.grabberMech = grabberMech;
    this.manualSpeedControl = manualSpeedControl;
    this.decimalFormat = decimalFormat;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString(
        "Grabber State",
        getState().name());

    SmartDashboard.putString(
        "Grabber Speed (RPM)",
        decimalFormat.format(getAverageVelocityPerSecond().getRotations() * 60.0));
  }

  public enum State {
    ENABLED, DISABLED
  }

  public State getState() {
    boolean isLimitSwitchOn = grabberMech.getLimitSwitchState();
    return isLimitSwitchOn ? State.ENABLED : State.DISABLED;
  }

  public Rotation2d getAverageVelocityPerSecond() {
    return grabberMech.getAverageVelocityRotationPerSecond();
  }

  public CommandBase createControlCommand() {

    Runnable control = new Runnable() {

      @Override
      public void run() {

        double velocityRPS = manualSpeedControl.getManualSpeed();
        Rotation2d velocityRotationsPerSecond = Rotation2d.fromRotations(velocityRPS);
        grabberMech.setAverageVelocityRotationsPerSecond(velocityRotationsPerSecond);
      }
    };

    return Commands.run(control, this);
  }

  public CommandBase createControlCommand(double velocityRPM) {

    Runnable control = new Runnable() {

      @Override
      public void run() {
        double velocityRPS = velocityRPM / 60.0;
        Rotation2d velocityRotationsPerSecond = Rotation2d.fromRotations(velocityRPS);
        grabberMech.setAverageVelocityRotationsPerSecond(velocityRotationsPerSecond);
      }
    };

    return Commands.runOnce(control, this);
  }

  public CommandBase createStopCommand() {
    Runnable control = new Runnable() {

      @Override
      public void run() {
        grabberMech.stop();
      }
    };

    return Commands.runOnce(control, this);
  }
}
