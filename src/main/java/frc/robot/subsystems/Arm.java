// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix.led.CANdle;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.framework.controls.ControllerAxis;
import frc.robot.framework.controls.HalfControllerAxis;
import frc.robot.framework.controls.ManualSpeedControl;
import frc.robot.framework.controls.PIDControllerFactory;
import frc.robot.framework.lighting.CANdleFactory;
import frc.robot.framework.lighting.LEDLights;
import frc.robot.framework.lighting.animations.LEDAnimation;
import frc.robot.framework.lighting.animations.multiColor.Fire;
import frc.robot.framework.lighting.animations.multiColor.RGBFade;
import frc.robot.framework.lighting.animations.multiColor.Rainbow;
import frc.robot.framework.lighting.animations.singleColor.ColorFlow;
import frc.robot.framework.lighting.animations.singleColor.Larson;
import frc.robot.framework.lighting.animations.singleColor.SingleColorForever;
import frc.robot.framework.lighting.animations.singleColor.SingleFade;
import frc.robot.framework.lighting.animations.singleColor.Strobe;
import frc.robot.framework.lighting.animations.singleColor.TwinkleOnOrOff;
import frc.robot.framework.mechanisms.LinearMech;
import frc.robot.framework.mechanisms.RotationMech;
import frc.robot.framework.mechanismsAdvanced.DualRotationMechWithLimitSwitch;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.motors.SparkMaxFactory;
import frc.robot.framework.pneumatics.DarthMaulCylinder;
import frc.robot.framework.pneumatics.JediCylinder;
import frc.robot.framework.sensors.AngularPositionSensor;
import frc.robot.framework.sensors.LinearPositionSensor;
import frc.robot.framework.sensors.TimeOfFlightFactory;
import frc.robot.framework.sensors.WPI_TimeOfFlightFactory;
import frc.robot.framework.sensors.WPI_TimeOfFlightFactory.WPI_TimeOfFlight;
import frc.robot.framework.switches.LimitSwitch;

public class Arm extends SubsystemBase {

  private final LinearMech elevatorMech;
  private final ManualSpeedControl elevatorManualSpeedControl;
  private final PIDController elevatorPIDController;
  private final MechanismLigament2d elevatorMech2d;
  private final DecimalFormat decimalFormat;
  private final RotationMech elbowMech;
  private final AngularPositionSensor elbowSensor;
  private final ManualSpeedControl elbowManualSpeedControl;
  private final PIDController elbowPIDController;
  private final MechanismLigament2d elbowMech2d;
  private final DualRotationMechWithLimitSwitch grabberMech;
  private final ManualSpeedControl grabberManualControl;
  private final DarthMaulCylinder elevatorTilt;
  private final MechanismLigament2d fakeElevatorMech2d;
  private double tiltAngleDegreesSim = 0;
  private final JediCylinder claw;

  private final LEDLights ledLights;

  public enum GrabberState {
    ENABLED, DISABLED
  }

  public enum ElevatorTiltState {
    NONE, TWO, SIX, EIGHT
  }

  public enum ClawState {
    CUBE, CONE
  }

  public Arm(
      CommandXboxController operatorController,
      MechanismLigament2d elevatorMech2d,
      MechanismLigament2d elbowMech2d,
      MechanismLigament2d fakeElevatorMech2d) {

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

    this.elbowManualSpeedControl = new ManualSpeedControl(
        armUpDownAxis,
        Constants.Robot.Arm.ElbowConstants.Control.maxManualSpeedRotationsPerSecond);

    this.elbowMech2d = elbowMech2d;

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

    Solenoid tiltSolenoid1 = new Solenoid(
        Constants.Robot.Arm.Pneumatics.pneumaticsModuleID,
        PneumaticsModuleType.REVPH,
        Constants.Robot.Arm.Elevator.Tilt.solenoid1Channel);

    Solenoid tiltSolenoid2 = new Solenoid(
        Constants.Robot.Arm.Pneumatics.pneumaticsModuleID,
        PneumaticsModuleType.REVPH,
        Constants.Robot.Arm.Elevator.Tilt.solenoid2Channel);

    JediCylinder frontCylinder = JediCylinder.getSingleSolenoidBased(tiltSolenoid1);
    JediCylinder backCylinder = JediCylinder.getSingleSolenoidBased(tiltSolenoid2);

    this.elevatorTilt = new DarthMaulCylinder(
        frontCylinder,
        backCylinder);

    this.fakeElevatorMech2d = fakeElevatorMech2d;

    DoubleSolenoid grabberClawDoubleSolenoid = new DoubleSolenoid(
        Constants.Robot.Arm.Pneumatics.pneumaticsModuleID,
        PneumaticsModuleType.REVPH,
        Constants.Robot.Arm.Claw.Control.doubleSolenoidChannel1,
        Constants.Robot.Arm.Claw.Control.doubleSolenoidChannel2);

    this.claw = JediCylinder.getDoubleSolenoidBased(grabberClawDoubleSolenoid);

    CANdle caNdle = CANdleFactory.create(
        Constants.Robot.Arm.LEDS.Candle.deviceId,
        Constants.Robot.Arm.LEDS.Candle.canbus,
        Constants.Robot.Arm.LEDS.Candle.ledCount,
        Constants.Robot.Arm.LEDS.Candle.ledStripType,
        Constants.Robot.Arm.LEDS.Candle.brightness,
        Constants.Robot.Arm.LEDS.Candle.disableWhenLOS,
        Constants.Robot.Arm.LEDS.Candle.disableWhenRunning,
        Constants.Robot.Arm.LEDS.Candle.vBatOutputMode,
        Constants.Robot.Arm.LEDS.Candle.dutyCyclePrcnt);

    this.ledLights = LEDLights.create(caNdle);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Suppliers
  // Elevator
  public double getElevatorPosition() {
    return elevatorMech.getPositionMeters();
  }

  public double getElevatorPositionFromSensor() {
    return elevatorMech.getLinearPositionFromSensor();
  }

  public double getElevatorVelocity() {
    return elevatorMech.getSpeedMetersPerSecond();
  }

  public boolean elevatorAtSetPoint() {
    return elevatorPIDController.atSetpoint();
  }

  public ManualSpeedControl getElevatorManualSpeedControl(){
    return elevatorManualSpeedControl;
  }

  // Elbow
  public double getElbowPosition() {
    return elbowMech.getPositionRotations().getRotations();
  }

  public double getElbowPositionFromSensor() {
    return this.elbowMech.getAngularPostionFromSensor().getRotations();
  }

  public double getElbowVelocity() {
    return elbowMech.getVelocityRotationPerSecond().getRotations();
  }

  public boolean elbowAtSetPoint() {
    return elbowPIDController.atSetpoint();
  }

  public ManualSpeedControl getElbowManualSpeedControl(){
    return elbowManualSpeedControl;
  }

  // Grabber
  public GrabberState getGrabberState() {
    boolean isLimitSwitchOn = grabberMech.getLimitSwitchState();
    return isLimitSwitchOn ? GrabberState.ENABLED : GrabberState.DISABLED;
  }

  public Rotation2d getAverageVelocityPerSecond() {
    return grabberMech.getAverageVelocityRotationPerSecond();
  }

  public ManualSpeedControl getGrabbManualSpeedControl(){
    return grabberManualControl;
  }

  // Elevator Tilt
  public ElevatorTiltState getElevatorTiltState() {
    DarthMaulCylinder.State darthMaulCylinderState = elevatorTilt.getDarthMaulCylinderState();
    if (darthMaulCylinderState == DarthMaulCylinder.State.NoLegs) {
      return ElevatorTiltState.NONE;
    } else if (darthMaulCylinderState == DarthMaulCylinder.State.ShortSaber) {
      return ElevatorTiltState.TWO;
    } else if (darthMaulCylinderState == DarthMaulCylinder.State.Tatooine) {
      return ElevatorTiltState.SIX;
    } else {
      return ElevatorTiltState.EIGHT;
    }
  }

  // Claw
  public ClawState getClawState() {
    JediCylinder.State jediCylinderState = this.claw.getExtendedState();
    return jediCylinderState == JediCylinder.State.Extended ? ClawState.CUBE : ClawState.CONE;
  }

  public boolean hasCone(){
    return this.claw.getExtendedState() == JediCylinder.State.Retracted;
  }

  //LEDLights
  public LEDAnimation getLEDAnimation(){
    return this.ledLights.getLEDAnimation();
  }

  // Consumers
  // Elevator
  public void setElevatorVelocity(double speedMetersPerSecond) {
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

  public void setElevatorSetpoint(double elevatorSetPoint) {
    elevatorPIDController.setSetpoint(elevatorSetPoint);
  }

  // Elbow
  public void setElbowVelocity(double rotationsPerSecond) {

    if (Math.abs(rotationsPerSecond) >= 0.68) {
      rotationsPerSecond = Math.signum(rotationsPerSecond) * 0.68;
    }

    if (RobotBase.isSimulation()) {
      rotationsPerSecond += 0.01;
    }
    Rotation2d rps = Rotation2d.fromRotations(rotationsPerSecond);
    this.elbowMech.setVelocityRotationsPerSecond(rps);
  }

  public void setElbowSetPoint(double elbowSetPoint) {
    elbowPIDController.setSetpoint(elbowSetPoint);
  }

  // Grabber
  public void setAverageVelocityPerSecond(double rotationsPerSecond) {
    Rotation2d rps = Rotation2d.fromRotations(rotationsPerSecond);
    if (getGrabberState() == GrabberState.ENABLED) {
      grabberMech.setAverageVelocityRotationsPerSecond(rps);
    } else {
      grabberMech.setAverageVelocityRotationsPerSecond(new Rotation2d());
    }
  }

  // Elevator Tilt
  public void setElevatorTiltState(ElevatorTiltState elevatorTiltState) {
    if (elevatorTiltState == ElevatorTiltState.NONE) {
      elevatorTilt.setDarthMaulCylinderState(DarthMaulCylinder.State.NoLegs);
    } else if (elevatorTiltState == ElevatorTiltState.TWO) {
      elevatorTilt.setDarthMaulCylinderState(DarthMaulCylinder.State.ShortSaber);
    } else if (elevatorTiltState == ElevatorTiltState.SIX) {
      elevatorTilt.setDarthMaulCylinderState(DarthMaulCylinder.State.Tatooine);
    } else {
      elevatorTilt.setDarthMaulCylinderState(DarthMaulCylinder.State.DuelOfTheFates);
    }
  }

  // Claw
  public void setClawState(ClawState clawState) {
    JediCylinder.State jediCylinderState = clawState == ClawState.CONE ? JediCylinder.State.Retracted
        : JediCylinder.State.Extended;
    claw.setExtendedState(jediCylinderState);
  }

  // LEDLights
  public void setLEDAnimation(LEDAnimation ledAnimation) {
    if (ledAnimation instanceof SingleColorForever) {
      ledLights.setLEDAnimation((SingleColorForever) ledAnimation);
    } else if (ledAnimation instanceof SingleFade) {
      ledLights.setLEDAnimation((SingleFade) ledAnimation);
    } else if (ledAnimation instanceof ColorFlow) {
      ledLights.setLEDAnimation((ColorFlow) ledAnimation);
    } else if (ledAnimation instanceof Larson) {
      ledLights.setLEDAnimation((Larson) ledAnimation);
    } else if (ledAnimation instanceof Strobe) {
      ledLights.setLEDAnimation((Strobe) ledAnimation);
    } else if (ledAnimation instanceof TwinkleOnOrOff) {
      ledLights.setLEDAnimation((TwinkleOnOrOff) ledAnimation);
    } else if (ledAnimation instanceof RGBFade) {
      ledLights.setLEDAnimation((RGBFade) ledAnimation);
    } else if (ledAnimation instanceof Rainbow) {
      ledLights.setLEDAnimation((Rainbow) ledAnimation);
    } else if (ledAnimation instanceof Fire) {
      ledLights.setLEDAnimation((Fire) ledAnimation);
    }
  }

  //Runnables
  public void clearLEDLights(){
    ledLights.clearLEDAnimations();
  }

  // Functions
  public double calculateElevatorSpeed(double currentElevatorPosition) {
    return elevatorPIDController.calculate(currentElevatorPosition);
  }

  public double calculateElbowSpeed(double currentElbowPosition) {
    return elevatorPIDController.calculate(currentElbowPosition);
  }
}
