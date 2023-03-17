// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting.animations.multiColor;

import frc.robot.framework.lighting.animations.Animation;

/** Add your docs here. */
public class RGBFade extends Animation {

    private final double brightness;
    private final double speed;

    public RGBFade(int ledStart, int numLed, double brightness, double speed) {
        super(ledStart, numLed);
        this.brightness = brightness;
        this.speed = speed;
    }

    public double getBrightness() {
        return brightness;
    }

    public double getSpeed() {
        return speed;
    }
}
