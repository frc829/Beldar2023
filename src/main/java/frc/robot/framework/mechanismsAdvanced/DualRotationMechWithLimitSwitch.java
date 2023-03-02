// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.mechanismsAdvanced;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.framework.mechanisms.RotationMech;
import frc.robot.framework.switches.LimitSwitch;

public interface DualRotationMechWithLimitSwitch {

  public boolean getLimitSwitchState();
  public Rotation2d getAverageVelocityRotationPerSecond();
  public void setAverageVelocityRotationsPerSecond(Rotation2d velocityRotation2dPerSecond);
  public void stop();

  public static DualRotationMechWithLimitSwitch create(
      RotationMech leftIntakeMech,
      RotationMech rightIntakeMech,
      LimitSwitch limitSwitch) {

    return new DualRotationMechWithLimitSwitch() {

      @Override
      public Rotation2d getAverageVelocityRotationPerSecond() {
        Rotation2d leftIntakeMechRotationsPerSecond = leftIntakeMech.getVelocityRotationPerSecond();
        Rotation2d rightIntakeMechRotationsPerSecond = rightIntakeMech.getVelocityRotationPerSecond();
        double leftIntakeMechRotationsPerSecondValue = -leftIntakeMechRotationsPerSecond.getRotations();
        double rightIntakeMechRotationsPerSecondValue = rightIntakeMechRotationsPerSecond.getRotations();
        double averageRotationsPerSecondValue = (leftIntakeMechRotationsPerSecondValue
            + rightIntakeMechRotationsPerSecondValue) / 2;
        return Rotation2d.fromRotations(averageRotationsPerSecondValue);
      }

      @Override
      public void setAverageVelocityRotationsPerSecond(Rotation2d velocityRotation2dPerSecond) {

        if(limitSwitch.isOn() && velocityRotation2dPerSecond.getRotations() > 0){
          stop();
        }
        else{
          leftIntakeMech.setVelocityRotationsPerSecond(velocityRotation2dPerSecond.unaryMinus());
          rightIntakeMech.setVelocityRotationsPerSecond(velocityRotation2dPerSecond);
        }


      }

      @Override
      public void stop() {
        leftIntakeMech.stop();
        rightIntakeMech.stop();
      }

      @Override
      public boolean getLimitSwitchState() {
        return limitSwitch.isOn();
      }
    };
  }
}
