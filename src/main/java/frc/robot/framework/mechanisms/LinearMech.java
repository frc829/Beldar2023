// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.sensors.LinearPositionSensor;

public interface LinearMech {

  /**
   * Returns the position of the mechanism in meters.
   * 
   * @return mechanism position in meters
   */
  public double getPositionMeters();

  /**
   * Returns the mechanism speed in meters per second.
   * 
   * @return mechanism speed in meters per second
   */
  public double getSpeedMetersPerSecond();

  /**
   * Sets the mechanism speed in meters per second
   * 
   * @param speedMetersPerSecond mechanism speed in meters per second
   */
  public void setMechanismSpeedMetersPerSecond(double speedMetersPerSecond);

  /**
   * Sets the position of the mechanism's motor encoder in meters.
   * 
   * @param mechanismPositionMeters position of the motor encoder to set
   */
  public void setMechanismPositionEncoder(double mechanismPositionMeters);

  public void stop();

  public double getRotationConversion();

  public double getLinearConversion();

  public double getLinearPositionFromSensor();

  public void setLinearPositionFromSensor();

  public static LinearMech create(
      Motor motor,
      double motorToMechConversion,
      double rotationMechToLinearConversion) {

    return LinearMech.create(
        motor,
        motorToMechConversion,
        rotationMechToLinearConversion,
        null);

  }

  public static LinearMech create(
      Motor motor,
      double motorToMechConversion,
      double rotationMechToLinearConversion,
      LinearPositionSensor linearPositionSensor) {

    LinearMech linearMech = new LinearMech() {
      /**
       * Returns the position of the mechanism in meters.
       * 
       * @return mechanism position in meters
       */
      public double getPositionMeters() {
        return motor.getPositionRotations().times(motorToMechConversion).times(rotationMechToLinearConversion).getRotations();
      }

      /**
       * Returns the mechanism speed in meters per second.
       * 
       * @return mechanism speed in meters per second
       */
      public double getSpeedMetersPerSecond() {
        return motor.getVelocityRotationPerSecond().times(motorToMechConversion).times(rotationMechToLinearConversion).getRotations();
      }

      /**
       * Sets the mechanism speed in meters per second
       * 
       * @param speedMetersPerSecond mechanism speed in meters per second
       */
      public void setMechanismSpeedMetersPerSecond(double speedMetersPerSecond) {
        double rotationsPerSecondValue = speedMetersPerSecond / rotationMechToLinearConversion / motorToMechConversion;
        Rotation2d rotationsPerSecond = Rotation2d.fromRotations(rotationsPerSecondValue);
        motor.setVelocityRotationsPerSecond(rotationsPerSecond);
      }

      /**
       * Sets the position of the mechanism's motor encoder in meters.
       * 
       * @param mechanismPositionMeters position of the motor encoder to set
       */
      public void setMechanismPositionEncoder(double mechanismPositionMeters) {
        double mechanismRotationsValue = mechanismPositionMeters / rotationMechToLinearConversion / motorToMechConversion;
        Rotation2d mechanismRotations = Rotation2d.fromRotations(mechanismRotationsValue);
        motor.setEncoderPosition(mechanismRotations);
      }

      public void stop() {
        motor.stop();
      }

      @Override
      public double getRotationConversion() {
        return motorToMechConversion;
      }

      @Override
      public double getLinearConversion() {
        return rotationMechToLinearConversion;
      }

      @Override
      public void setLinearPositionFromSensor() {
        if (linearPositionSensor != null) {
          double linearPositionFromSensor = linearPositionSensor.getPosition();
          setMechanismPositionEncoder(linearPositionFromSensor);
        }
      }

      @Override
      public double getLinearPositionFromSensor() {
        if (linearPositionSensor != null) {
          return linearPositionSensor.getPosition();
        } else {
          return Double.NaN;
        }
      }
    };

    linearMech.setLinearPositionFromSensor();
    return linearMech;
  }

}
