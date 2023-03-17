// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting.animations.singleColor;

/** Add your docs here. */
public class SingleFade extends SingleColorForever {

    private final double speed;

    public SingleFade(int ledStart, int numLed, int red, int green, int blue, double speed) {
        super(ledStart, numLed, red, green, blue);
        this.speed = speed;
    }

    public SingleFade(int ledStart, int numLed, int red, int green, int blue, int white, double speed) {
        super(ledStart, numLed, red, green, blue, white);
        this.speed = speed;
    }

    public double getSpeed() {
        return this.speed;
    }

}
