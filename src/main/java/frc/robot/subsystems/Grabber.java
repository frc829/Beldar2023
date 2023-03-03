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
import frc.robot.framework.mechanisms.MotorReduction;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.switches.LimitSwitch;

public class Grabber extends SubsystemBase {

  public enum State {
    ENABLED, DISABLED
  }

  private final Motor leftIntakeMotor;
  private final Motor rightIntakeMotor;
  private final MotorReduction leftMotorReduction;
  private final MotorReduction rightMotorReduction;
  private final LimitSwitch limitSwitch;
  private final ManualSpeedControl manualSpeedControl;
  private final DecimalFormat decimalFormat;

  public Grabber(
      Motor leftIntakeMotor,
      Motor rightIntakeMotor,
      MotorReduction leftMotorReduction,
      MotorReduction rightMotorReduction,
      LimitSwitch limitSwitch,
      ManualSpeedControl manualSpeedControl,
      DecimalFormat decimalFormat) {

    this.leftIntakeMotor = leftIntakeMotor;
    this.rightIntakeMotor = rightIntakeMotor;
    this.leftMotorReduction = leftMotorReduction;
    this.rightMotorReduction = rightMotorReduction;
    this.limitSwitch = limitSwitch;
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
        decimalFormat.format(getAverageVelocityPerSecond() * 60.0));
  }

  // Suppliers
  public State getState() {
    boolean isLimitSwitchOn = this.limitSwitch.isOn();
    return isLimitSwitchOn ? State.ENABLED : State.DISABLED;
  }

  public double getAverageVelocityPerSecond() {
    Rotation2d leftIntakeMotorRPS = leftIntakeMotor.getVelocity();
    Rotation2d rightIntakeMotorRPS = rightIntakeMotor.getVelocity();
    Rotation2d leftMechRPS = leftMotorReduction.reduceMotorRotations(leftIntakeMotorRPS);
    Rotation2d rightMechRPS = rightMotorReduction.reduceMotorRotations(rightIntakeMotorRPS);
    double leftMechRPSValue = leftMechRPS.getRotations();
    double rightMechRPSValue = rightMechRPS.getRotations();
    double averageRPS = (leftMechRPSValue + rightMechRPSValue) / 2;
    return averageRPS;
  }

  // Consumers
  public void setAverageVelocityPerSecond(double averageVelocityRotationsPerSecond) {
    if (getState() == State.DISABLED) {
      leftIntakeMotor.setVelocity(new Rotation2d());
      rightIntakeMotor.setVelocity(new Rotation2d());
    } else {
      Rotation2d leftMechRPS = Rotation2d.fromRotations(averageVelocityRotationsPerSecond);
      Rotation2d rightMechRPS = Rotation2d.fromRotations(averageVelocityRotationsPerSecond);
      Rotation2d leftIntakeMotorRPS = leftMotorReduction.expandMechanismRotations(leftMechRPS);
      Rotation2d rightIntakeMotorRPS = rightMotorReduction.expandMechanismRotations(rightMechRPS);
      leftIntakeMotor.setVelocity(leftIntakeMotorRPS);
      rightIntakeMotor.setVelocity(rightIntakeMotorRPS);
    }

  }

  // Commands
  public CommandBase createManualControlCommand() {

    CommandBase setIntakes = new CommandBase() {
      @Override
      public void execute() {
        double speedRPS = manualSpeedControl.getManualSpeed();
        setAverageVelocityPerSecond(speedRPS);
      }
    };
    return setIntakes;
  }

  public CommandBase createSetSpeedCommand(double speedRPM) {
    CommandBase setIntakes = new CommandBase() {
      @Override
      public void initialize() {
        setAverageVelocityPerSecond(speedRPM);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    return setIntakes;
  }

  public CommandBase createStopCommand() {
    CommandBase stopLeftIntake = Commands.runOnce(leftIntakeMotor::stop);
    CommandBase stopRightIntake = Commands.runOnce(rightIntakeMotor::stop);
    return Commands.parallel(stopLeftIntake, stopRightIntake);
  }
}
