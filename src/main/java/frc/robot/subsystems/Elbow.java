// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.framework.mechanisms.RotationMech;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.motors.SparkMaxFactory;
import frc.robot.framework.sensors.AngularPositionSensor;

public class Elbow extends SubsystemBase {

  private final RotationMech elbowMech;
  private final AngularPositionSensor elbowSensor;
  private final ManualSpeedControl manualSpeedControl;
  private final PIDController elbowPIDController;
  private final MechanismLigament2d elbowMech2d;
  private final DecimalFormat decimalFormat;

  public Elbow(
      CommandXboxController operatorController,
      MechanismLigament2d elbowMech2d) {

    this.elbowPIDController = PIDControllerFactory.create(
        Constants.Robot.Arm.ElbowConstants.Control.PID.kP,
        Constants.Robot.Arm.ElbowConstants.Control.PID.kI,
        Constants.Robot.Arm.ElbowConstants.Control.PID.kD,
        Constants.Robot.Arm.ElbowConstants.Control.PID.tolerance,
        Constants.Robot.Arm.ElbowConstants.Control.PID.minimumInput,
        Constants.Robot.Arm.ElbowConstants.Control.PID.maximumInput);

    CANSparkMax elbowMotorSparkMax = SparkMaxFactory.create(
        Constants.Robot.Arm.ElbowConstants.MotorConfig.deviceId,
        Constants.Robot.Arm.ElbowConstants.MotorConfig.revMotor,
        Constants.Robot.Arm.ElbowConstants.MotorConfig.isInverted,
        Constants.Robot.Arm.ElbowConstants.MotorConfig.idleMode,
        Constants.Robot.Arm.ElbowConstants.MotorConfig.velocityKP,
        Constants.Robot.Arm.ElbowConstants.MotorConfig.velocityKI,
        Constants.Robot.Arm.ElbowConstants.MotorConfig.velocityKD,
        Constants.Robot.Arm.ElbowConstants.MotorConfig.velocityKF);

    Motor elbowMotor = Motor.create(
        elbowMotorSparkMax,
        Constants.Robot.Arm.ElbowConstants.MotorConfig.revMotor);

    this.elbowSensor = AngularPositionSensor.getREVThroughBoreEncoder(
        Constants.Robot.Arm.ElbowConstants.Sensor.dioChannel,
        Constants.Robot.Arm.ElbowConstants.Sensor.offsetDegrees,
        elbowMotor,
        Constants.Robot.Arm.ElbowConstants.MechConfig.motorToMechConversion);

    this.elbowMech = RotationMech.create(
        elbowMotor,
        Constants.Robot.Arm.ElbowConstants.MechConfig.motorToMechConversion,
        elbowSensor);

    ControllerAxis armUpDownAxis = ControllerAxis.getAxisControl(
        operatorController,
        XboxController.Axis.kRightY,
        false,
        Constants.OperatorConstants.ElevatorManualControls.kDeadband);

    this.manualSpeedControl = new ManualSpeedControl(
        armUpDownAxis,
        Constants.Robot.Arm.ElbowConstants.Control.maxManualSpeedRotationsPerSecond);

    this.elbowMech2d = elbowMech2d;
    this.decimalFormat = new DecimalFormat("###.###");
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

  // Suppliers
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

  public boolean atSetpoint() {
    return elbowPIDController.atSetpoint();
  }

  public boolean manualSpeedNonZero() {
    return this.manualSpeedControl.getManualSpeed() != 0;
  }

  // Runnables

  public void setManualControlTrigger() {
    Trigger elbowManualControlTrigger = new Trigger(this::manualSpeedNonZero);
    CommandBase elbowManualControlCommand = this.createControlCommand();
    elbowManualControlTrigger.whileTrue(elbowManualControlCommand);
  }

  private void stop() {
    this.elbowMech.stop();
  }

  // Consumers
  private void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    if (RobotBase.isSimulation()) {
      rotationsPerSecond += 0.01;
    }
    Rotation2d rps = Rotation2d.fromRotations(rotationsPerSecond);
    this.elbowMech.setVelocityRotationsPerSecond(rps);
  }

  // Commands
  public CommandBase createHoldCommand() {

    PIDCommand pidCommand = new PIDCommand(
        elbowPIDController,
        this::getPositionRotations,
        getPositionRotations(),
        this::setVelocityRotationsPerSecond,
        this) {
      @Override
      public void initialize() {
        double setPoint = getPositionRotations();
        this.m_setpoint = () -> setPoint;
        SmartDashboard.putString("Elbow Command Current", "Hold at " + this.m_setpoint.getAsDouble() * 360.0 + " degs");
        super.initialize();
      }

      @Override
      public void execute() {
        double output = m_controller.calculate(m_measurement.getAsDouble(), m_setpoint.getAsDouble());
        SmartDashboard.putNumber("CurrentElbowSetpoint", m_setpoint.getAsDouble());
        SmartDashboard.putNumber("Hold Elbow Output", output);
        m_useOutput.accept(output);
      }
    };

    return pidCommand;
  }

  public CommandBase createControlCommand() {

    CommandBase manualControlCommand = new CommandBase() {

      @Override
      public void initialize() {
        SmartDashboard.putString("Elbow Command Current", "Manual Control");
      }

      @Override
      public void execute() {
        double manualSpeed = manualSpeedControl.getManualSpeed();
        setVelocityRotationsPerSecond(manualSpeed);
      }

    };

    manualControlCommand.addRequirements(this);
    return manualControlCommand;
  }

  public CommandBase createControlCommand(double positionDegrees) {

    PIDCommand pidCommand = new PIDCommand(
        elbowPIDController,
        this::getPositionRotations,
        positionDegrees / 360.0,
        this::setVelocityRotationsPerSecond,
        this) {

      @Override
      public void initialize() {
        SmartDashboard.putString("Elbow Command Current", "Go To " + positionDegrees + " degrees");
        super.initialize();
      }
    };

    return pidCommand;

  }

  public CommandBase createStopCommand() {
    CommandBase stopCommand = new CommandBase() {
      @Override
      public void initialize() {
        SmartDashboard.putString("Elbow Command Current", "Stop");
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
