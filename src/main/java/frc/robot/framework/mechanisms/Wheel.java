// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface Wheel {

    public double transformRotationToLinear(Rotation2d angularValue);

    public Rotation2d transformLinearToRotation(double linearValue);

    public enum AndyMarkSprocket {
        TwentyTwo25(1.76);

        private double value;

        private AndyMarkSprocket(double value) {
            this.value = value;
        }
    }

    public static Wheel create(AndyMarkSprocket andyMarkSprocket) {

        return new Wheel() {

            @Override
            public double transformRotationToLinear(Rotation2d angularValue) {
                return angularValue.times(andyMarkSprocket.value).getRotations();
            }

            @Override
            public Rotation2d transformLinearToRotation(double linearValue) {
                return Rotation2d.fromRotations(linearValue / andyMarkSprocket.value);
            }
        };
    }
}
