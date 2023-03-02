// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting;

/** Add your docs here. */
public class LEDConfig {
    public final int red; 
    public final int green; 
    public final int blue; 
    public final int white;
    public final int startIndex; 
    public final int count;

    public LEDConfig(int red, int green, int blue, int white, int startIndex, int count){
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.white = white;
        this.startIndex = startIndex;
        this.count = count;
    }
}
