// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting.animations.singleColor;

/** Add your docs here. */
public class Larson extends SingleFade {

    public enum Bounce {
        Front,
        Center,
        Back
    }

    public final int size;

    public final Bounce bounce;

    public Larson(int ledStart, int numLed, int red, int green, int blue, double speed, Bounce bounce,
            int size) {
        super(ledStart, numLed, red, green, blue, speed);
        this.size = size;
        this.bounce = bounce;
    }

    public Larson(int ledStart, int numLed, int red, int green, int blue, int white, double speed,
            Bounce bounce, int size) {
        super(ledStart, numLed, red, green, blue, white, speed);
        this.size = size;
        this.bounce = bounce;
    }

    public int getSize() {
        return this.size;
    }

    public Bounce getBounce() {
        return this.bounce;
    }

}
