// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting.animations.multiColor;

/** Add your docs here. */
public class Fire extends Rainbow {

    private final double sparkling;
    private final double cooling;

    public Fire(int ledStart, int numLed, double brightness, double speed, boolean isReversed,
            double sparkling, double cooling) {
        super(ledStart, numLed, brightness, speed, isReversed);
        this.sparkling = sparkling;
        this.cooling = cooling;
    }

    public double getSparkling() {
        return this.sparkling;
    }

    public double getCooling() {
        return this.cooling;
    }
}
