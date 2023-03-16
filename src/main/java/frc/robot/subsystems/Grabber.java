// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;
import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.framework.controls.ControllerAxis;
import frc.robot.framework.controls.HalfControllerAxis;
import frc.robot.framework.controls.ManualSpeedControl;
import frc.robot.framework.mechanisms.RotationMech;
import frc.robot.framework.mechanismsAdvanced.DualRotationMechWithLimitSwitch;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.motors.SparkMaxFactory;
import frc.robot.framework.sensors.LinearPositionSensor;
import frc.robot.framework.sensors.TimeOfFlightFactory;
import frc.robot.framework.sensors.WPI_TimeOfFlightFactory;
import frc.robot.framework.sensors.WPI_TimeOfFlightFactory.WPI_TimeOfFlight;
import frc.robot.framework.switches.LimitSwitch;

public class Grabber extends SubsystemBase {

  private final DualRotationMechWithLimitSwitch grabberMech;
  private final ManualSpeedControl grabberManualControl;
  private final DecimalFormat decimalFormat;

  public Grabber(CommandXboxController operatorController) {

    CANSparkMax leftGripperMotorSparkMax = SparkMaxFactory.create(
        Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.deviceId,
        Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.revMotor,
        Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.isInverted,
        Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.idleMode,
        Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.velocityKP,
        Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.velocityKI,
        Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.velocityKD,
        Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.velocityKF);
    CANSparkMax rightGripperMotorSparkMax = SparkMaxFactory.create(
        Constants.Robot.Arm.Claw.RightClaw.MotorConfig.deviceId,
        Constants.Robot.Arm.Claw.RightClaw.MotorConfig.revMotor,
        Constants.Robot.Arm.Claw.RightClaw.MotorConfig.isInverted,
        Constants.Robot.Arm.Claw.RightClaw.MotorConfig.idleMode,
        Constants.Robot.Arm.Claw.RightClaw.MotorConfig.velocityKP,
        Constants.Robot.Arm.Claw.RightClaw.MotorConfig.velocityKI,
        Constants.Robot.Arm.Claw.RightClaw.MotorConfig.velocityKD,
        Constants.Robot.Arm.Claw.RightClaw.MotorConfig.velocityKF);

    Motor leftGripperMotor = Motor.create(
        leftGripperMotorSparkMax,
        Constants.Robot.Arm.Claw.LeftClaw.MotorConfig.revMotor);
    Motor rightGripperMotor = Motor.create(
        rightGripperMotorSparkMax,
        Constants.Robot.Arm.Claw.RightClaw.MotorConfig.revMotor);

    RotationMech leftGripperMech = RotationMech.create(
        leftGripperMotor,
        Constants.Robot.Arm.Claw.LeftClaw.MechConfig.motorToMechConversion);

    RotationMech rightGripperMech = RotationMech.create(
        rightGripperMotor,
        Constants.Robot.Arm.Claw.RightClaw.MechConfig.motorToMechConversion);

    TimeOfFlight grabberTimeOfFlight = TimeOfFlightFactory.create(
        Constants.Robot.Arm.Claw.LimitSwitch.Sensor.positionSensorID,
        Constants.Robot.Arm.Claw.LimitSwitch.Sensor.mode,
        Constants.Robot.Arm.Claw.LimitSwitch.Sensor.sampleTime);

    WPI_TimeOfFlight grabberWPI_TimeOfFlight = WPI_TimeOfFlightFactory.create(
        grabberTimeOfFlight,
        Constants.Robot.Arm.Claw.LimitSwitch.Sensor.positionSensorID);

    LinearPositionSensor grabberTimeOfFlightSensor = LinearPositionSensor.create(
        Constants.Robot.Arm.Claw.LimitSwitch.Sensor.positionSensorID,
        grabberWPI_TimeOfFlight);

    LimitSwitch grabberLimitSwitch = LimitSwitch.create(
        grabberTimeOfFlightSensor,
        Constants.Robot.Arm.Claw.LimitSwitch.minTruePositionMeters,
        Constants.Robot.Arm.Claw.LimitSwitch.maxTruePositionMeters);

    DualRotationMechWithLimitSwitch grabberMech = DualRotationMechWithLimitSwitch.create(
        leftGripperMech,
        rightGripperMech,
        grabberLimitSwitch);

    HalfControllerAxis grabberPositiveRotation = HalfControllerAxis.getAxisControl(
        operatorController,
        HalfControllerAxis.PositiveOnlyAxisType.LeftTriggerAxis,
        false,
        Constants.OperatorConstants.ElevatorManualControls.kDeadband);

    HalfControllerAxis grabberNegativeRotation = HalfControllerAxis.getAxisControl(
        operatorController,
        HalfControllerAxis.PositiveOnlyAxisType.RightTriggerAxis,
        false,
        Constants.OperatorConstants.ElevatorManualControls.kDeadband);

    ControllerAxis grabberControlAxis = ControllerAxis.getAxisControl(grabberPositiveRotation,
        grabberNegativeRotation);

    ManualSpeedControl grabberManualControl = new ManualSpeedControl(
        grabberControlAxis,
        Constants.Robot.Arm.Claw.Control.maxManualSpeedRPS);

    this.grabberMech = grabberMech;
    this.grabberManualControl = grabberManualControl;
    this.decimalFormat = new DecimalFormat("###.###");
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

  public void setManualControlTrigger() {
    BooleanSupplier grabberManualControlSupplier = new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        return grabberManualControl.getManualSpeed() != 0;
      }

    };
    Trigger grabberManualControlTrigger = new Trigger(grabberManualControlSupplier);
    CommandBase grabberManualControlCommand = this.createControlCommand();
    grabberManualControlTrigger.whileTrue(grabberManualControlCommand);
  }

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
        double manualSpeed = grabberManualControl.getManualSpeed();
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
