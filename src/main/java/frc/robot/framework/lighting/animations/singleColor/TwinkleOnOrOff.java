// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting.animations.singleColor;

/** Add your docs here. */
public class TwinkleOnOrOff extends SingleFade {

    public final int divider;
    public final boolean isTwinkleOn;

    public TwinkleOnOrOff(int ledStart, int numLed, int red, int green, int blue, double speed, int divider,
            boolean isTwinkleOn) {
        super(ledStart, numLed, red, green, blue, speed);
        this.divider = divider;
        this.isTwinkleOn = isTwinkleOn;
    }

    public TwinkleOnOrOff(int ledStart, int numLed, int red, int green, int blue, int white, double speed,
            int divider, boolean isTwinkleOn) {
        super(ledStart, numLed, red, green, blue, white);
        this.divider = divider;
        this.isTwinkleOn = isTwinkleOn;
    }

    public int getDivider() {
        return this.divider;
    }

    public boolean getIsTwinkleOn() {
        return this.isTwinkleOn;
    }
}
