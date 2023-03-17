// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting.animations;

/** Add your docs here. */
public abstract class Animation {

    private final int ledStart;
    private final int numLed;

    public Animation(int ledStart, int numLed){
        this.ledStart = ledStart;
        this.numLed = numLed;
    }

    public int getLedStart(){
        return this.ledStart;
    }

    public int getNumLed(){
        return this.numLed;
    }

}
