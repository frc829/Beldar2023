// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  public enum State {
    ENABLED, DISABLED
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

  // Suppliers
  public State getState() {
    boolean isLimitSwitchOn = grabberMech.getLimitSwitchState();
    return isLimitSwitchOn ? State.ENABLED : State.DISABLED;
  }

  public Rotation2d getAverageVelocityPerSecond() {
    return grabberMech.getAverageVelocityRotationPerSecond();
  }

  // Runnables
  public void stop() {
    grabberMech.stop();
  }

  // Consumers
  public void setAverageVelocityPerSecond(double rotationsPerSecond) {
    Rotation2d rps = Rotation2d.fromRotations(rotationsPerSecond);
    grabberMech.setAverageVelocityRotationsPerSecond(rps);
  }

  // Commands
  public CommandBase createControlCommand() {

    CommandBase manualControlCommand = new CommandBase() {

      @Override
      public void initialize() {
        SmartDashboard.putString("Grabber Command Current", "Manual Control");
      }

      @Override
      public void execute() {
        double manualSpeed = manualSpeedControl.getManualSpeed();
        setAverageVelocityPerSecond(manualSpeed);
      }

    };

    manualControlCommand.addRequirements(this);
    return manualControlCommand;
  }

  public CommandBase createControlCommand(double velocityRPM) {

    CommandBase setGrabberSpeedCommand = new CommandBase() {

      @Override
      public void initialize() {
        SmartDashboard.putString("Grabber Command Current", "Set Speed: " + velocityRPM + " rpm");
      }

      @Override
      public void execute() {
        double rps = velocityRPM / 60.0;
        Rotation2d rotationsPerSecond = Rotation2d.fromRotations(rps);
        grabberMech.setAverageVelocityRotationsPerSecond(rotationsPerSecond);
      }

    };

    setGrabberSpeedCommand.addRequirements(this);

    return setGrabberSpeedCommand;

  }

  public CommandBase createPoofCommand(double velocityRPM) {

    CommandBase setGrabberSpeedCommand = new CommandBase() {

      @Override
      public void initialize() {
        SmartDashboard.putString("Grabber Command Current", "Poof Speed: " + velocityRPM + " rpm");
      }

      @Override
      public void execute() {
        double rps = velocityRPM / 60.0;
        Rotation2d rotationsPerSecond = Rotation2d.fromRotations(rps);
        grabberMech.setAverageVelocityRotationsPerSecond(rotationsPerSecond);
      }

      @Override
      public void end(boolean interrupted) {
        stop();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };

    setGrabberSpeedCommand.addRequirements(this);

    return setGrabberSpeedCommand;

  }

  public CommandBase createStopCommand() {

    CommandBase stopCommand = new CommandBase() {
      @Override
      public void initialize() {
        SmartDashboard.putString("Grabber Command Current", "Stop");
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
