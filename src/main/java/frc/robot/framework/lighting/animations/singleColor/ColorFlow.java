// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting.animations.singleColor;

/** Add your docs here. */
public class ColorFlow extends SingleFade {

    private final boolean isReversed;

    public ColorFlow(int ledStart, int numLed, int red, int green, int blue, double speed,
            boolean isReversed) {
        super(ledStart, numLed, red, green, blue, speed);
        this.isReversed = isReversed;
    }

    public ColorFlow(int ledStart, int numLed, int red, int green, int blue, int white, double speed,
            boolean isReversed) {
        super(ledStart, numLed, red, green, blue, white, speed);
        this.isReversed = isReversed;
    }

    public boolean getIsReversed() {
        return isReversed;
    }
}
