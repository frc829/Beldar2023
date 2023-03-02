// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.sensors.AngularPositionSensor;

/** Add your docs here. */
public interface RotationMech extends Motor {

    public double getRotationConversion();

    public Rotation2d getAngularPostionFromSensor();

    public void setEncoderPositionFromSensor();

    public static RotationMech create(Motor leftGripperMotor, double motorToMechConversion) {
        return RotationMech.create(
            leftGripperMotor, 
            motorToMechConversion,
            null);
    }

    public static RotationMech create(
            Motor motor,
            double rotationConversion,
            AngularPositionSensor angularPositionSensor) {

        RotationMech rotationMech = new RotationMech() {

            @Override
            public Rotation2d getPositionRotations() {
                return motor.getPositionRotations().times(rotationConversion);
            }

            @Override
            public Rotation2d getVelocityRotationPerSecond() {
                return motor.getVelocityRotationPerSecond().times(rotationConversion);
            }

            @Override
            public void setEncoderPosition(Rotation2d rotation2d) {
                motor.setEncoderPosition(rotation2d.div(rotationConversion));
            }

            @Override
            public void setVelocityRotationsPerSecond(Rotation2d velocityRotation2dPerSecond) {
                Rotation2d motorVelocityRotationsPerSecond = velocityRotation2dPerSecond.div(rotationConversion);

                motor.setVelocityRotationsPerSecond(motorVelocityRotationsPerSecond);
            }

            @Override
            public void stop() {
                motor.stop();
            }

            @Override
            public double getVoltage() {
                return motor.getVoltage();
            }

            @Override
            public DCMotor getDCMotor() {
                return motor.getDCMotor();
            }

            @Override
            public double getRotationConversion() {
                return rotationConversion;
            }

            @Override
            public Rotation2d getAngularPostionFromSensor() {
                if (angularPositionSensor != null) {
                    Rotation2d sensorRotations = angularPositionSensor.getAbsoluteAngle();
                    double sensorRotationsValue = sensorRotations.getRotations();
                    sensorRotationsValue %= 1;
                    sensorRotationsValue = sensorRotationsValue < 0 ? sensorRotationsValue + 1 : sensorRotationsValue;
                    return Rotation2d.fromRotations(sensorRotationsValue);
                } else {
                    return Rotation2d.fromRotations(Double.NaN);
                }
            }

            @Override
            public void setEncoderPositionFromSensor() {
                if (angularPositionSensor != null) {
                    setEncoderPosition(angularPositionSensor.getAbsoluteAngle());
                }
            }
        };

        rotationMech.setEncoderPositionFromSensor();
        return rotationMech;
    }



}
