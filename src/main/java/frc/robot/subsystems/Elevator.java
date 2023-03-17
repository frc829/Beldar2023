// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.framework.controls.ControllerAxis;
import frc.robot.framework.controls.ManualSpeedControl;
import frc.robot.framework.controls.PIDControllerFactory;
import frc.robot.framework.mechanisms.LinearMech;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.motors.SparkMaxFactory;
import frc.robot.framework.sensors.LinearPositionSensor;
import frc.robot.framework.sensors.TimeOfFlightFactory;
import frc.robot.framework.sensors.WPI_TimeOfFlightFactory;
import frc.robot.framework.sensors.WPI_TimeOfFlightFactory.WPI_TimeOfFlight;

public class Elevator extends SubsystemBase {

  private final LinearMech elevatorMech;
  private final ManualSpeedControl elevatorManualSpeedControl;
  private final PIDController elevatorPIDController;
  private final MechanismLigament2d elevatorMech2d;
  private final DecimalFormat decimalFormat;

  public Elevator(
      CommandXboxController operatorController,
      MechanismLigament2d elevatorMech2d) {

    ControllerAxis elevatorUpDownAxis = ControllerAxis.getAxisControl(
        operatorController,
        XboxController.Axis.kLeftY,
        true,
        Constants.OperatorConstants.ElevatorManualControls.kDeadband);

    ManualSpeedControl elevatorManualSpeedControl = new ManualSpeedControl(
        elevatorUpDownAxis,
        Constants.Robot.Arm.Elevator.Control.maxManualSpeedMPS);

    PIDController elevatorPIDController = PIDControllerFactory.create(
        Constants.Robot.Arm.Elevator.Control.PID.kP,
        Constants.Robot.Arm.Elevator.Control.PID.kI,
        Constants.Robot.Arm.Elevator.Control.PID.kD,
        Constants.Robot.Arm.Elevator.Control.PID.tolerance);

    CANSparkMax elevatorMotorSparkMax = SparkMaxFactory.create(
        Constants.Robot.Arm.Elevator.MotorConfig.deviceId,
        Constants.Robot.Arm.Elevator.MotorConfig.revMotor,
        Constants.Robot.Arm.Elevator.MotorConfig.isInverted,
        Constants.Robot.Arm.Elevator.MotorConfig.idleMode,
        Constants.Robot.Arm.Elevator.MotorConfig.velocityKP,
        Constants.Robot.Arm.Elevator.MotorConfig.velocityKI,
        Constants.Robot.Arm.Elevator.MotorConfig.velocityKD,
        Constants.Robot.Arm.Elevator.MotorConfig.velocityKF);

    Motor elevatorMotor = Motor.create(
        elevatorMotorSparkMax,
        Constants.Robot.Arm.Elevator.MotorConfig.revMotor);

    TimeOfFlight elevatorTimeOfFlight = TimeOfFlightFactory.create(
        Constants.Robot.Arm.Elevator.Sensor.positionSensorID,
        Constants.Robot.Arm.Elevator.Sensor.mode,
        Constants.Robot.Arm.Elevator.Sensor.sampleTime);

    WPI_TimeOfFlight elevatorWPI_TimeOfFlight = WPI_TimeOfFlightFactory.create(
        elevatorTimeOfFlight,
        Constants.Robot.Arm.Elevator.Sensor.positionSensorID,
        elevatorMotor,
        Constants.Robot.Arm.Elevator.MechConfig.motorToMechConversion,
        Constants.Robot.Arm.Elevator.MechConfig.rotationToLinearconversion,
        Constants.Robot.Arm.Elevator.Control.minPosition,
        Constants.Robot.Arm.Elevator.Control.maxPosition);

    LinearPositionSensor elevatorPositionSensor = LinearPositionSensor.create(
        Constants.Robot.Arm.Elevator.Sensor.positionSensorID,
        elevatorWPI_TimeOfFlight);

    this.elevatorMech = LinearMech.create(
        elevatorMotor,
        Constants.Robot.Arm.Elevator.MechConfig.motorToMechConversion,
        Constants.Robot.Arm.Elevator.MechConfig.rotationToLinearconversion,
        elevatorPositionSensor);

    this.elevatorManualSpeedControl = elevatorManualSpeedControl;
    this.elevatorPIDController = elevatorPIDController;
    this.elevatorMech2d = elevatorMech2d;
    this.decimalFormat = new DecimalFormat("###.###");
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
  public double getPosition() {
    return elevatorMech.getPositionMeters();
  }

  private double getVelocity() {
    return elevatorMech.getSpeedMetersPerSecond();
  }

  private double getPositionFromSensor() {
    return elevatorMech.getLinearPositionFromSensor();
  }

  public boolean atSetpoint() {
    return elevatorPIDController.atSetpoint();
  }

  public boolean manualSpeedNonZero() {
    return this.elevatorManualSpeedControl.getManualSpeed() != 0;
  }

  public PIDController getPIDController(){
    return this.elevatorPIDController;
  }

  // Runnable

  public void setManualControlTrigger() {
    Trigger elevatorManualControlTrigger = new Trigger(this::manualSpeedNonZero);
    CommandBase elevatorManualControlCommand = this.createControlCommand();
    elevatorManualControlTrigger.whileTrue(elevatorManualControlCommand);
  }

  private void stop() {
    this.elevatorMech.stop();
  }

  // Consumers
  public void setVelocity(double speedMetersPerSecond) {
    if (RobotBase.isSimulation()) {
      double gravitySim = 0;
      if (elevatorMech.getPositionMeters() > 0) {
        gravitySim = -0.1;
      } else {
        gravitySim = 0.0;
      }
      speedMetersPerSecond += gravitySim;
      if (speedMetersPerSecond < 0 && elevatorMech.getPositionMeters() <= 0) {
        speedMetersPerSecond = 0;
      }
    }
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
        double setPoint = getPosition();
        this.m_setpoint = () -> setPoint;
        SmartDashboard.putString("Elevator Command Current", "Hold at " + this.m_setpoint.getAsDouble() + " m");
        super.initialize();
      }

      @Override
      public void execute() {
        double output = m_controller.calculate(m_measurement.getAsDouble(), m_setpoint.getAsDouble());
        SmartDashboard.putNumber("CurrentElevatorSetpoint", m_setpoint.getAsDouble());
        SmartDashboard.putNumber("Hold Elevator Output", output);
        m_useOutput.accept(output);
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
        double manualSpeed = elevatorManualSpeedControl.getManualSpeed();
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
