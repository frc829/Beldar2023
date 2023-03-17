// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting.animations.multiColor;

/** Add your docs here. */
public class Rainbow extends RGBFade {

    private final boolean isReversed;

    public Rainbow(int ledStart, int numLed, double brightness, double speed, boolean isReversed) {
        super(ledStart, numLed, brightness, speed);
        this.isReversed = isReversed;
    }

    public boolean getIsReversed() {
        return isReversed;
    }
}
