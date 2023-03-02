// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.framework.lighting.LEDConfig;

/** Add your docs here. */
public class LEDLighting extends SubsystemBase {

    private final CANdle candle;
    private String currentAnimationName;
    private LEDConfig currentLEDConfig = null;

    public LEDLighting(
            int deviceID,
            String canbus,
            int ledCount) {
        this.candle = new CANdle(deviceID, canbus);
        this.candle.configLEDType(LEDStripType.GRB, 0);
        this.candle.configBrightnessScalar(1, 0);
        this.candle.configLOSBehavior(true, 0);
        this.candle.configStatusLedState(false, 0);
        this.candle.configVBatOutput(VBatOutputMode.Modulated);
        this.candle.modulateVBatOutput(1);
        this.currentAnimationName = "Off";

    }

    public void changeBrightness(double percent) {
        this.candle.configBrightnessScalar(percent, 0);
    }

    public void modulateBatteryOutput(double percent) {
        this.candle.modulateVBatOutput(percent);
    }

    public void setAnimations(String animationName, Animation... animations) {
        this.candle.animate(null);
        this.currentAnimationName = animationName;
        if (this.candle.getMaxSimultaneousAnimationCount() >= animations.length) {
            for (Animation animation : animations) {
                setAnimation(animation);
            }
        }
    }

    private void setAnimation(Animation animation) {
        this.candle.animate(animation);
    }

    public void animateOff() {
        this.currentAnimationName = "Off";
        this.candle.animate(null);
    }

    public void setLEDSequences(String animationName, LEDConfig... ledSequences) {
        this.candle.animate(null);
        this.currentAnimationName = animationName;
        for (LEDConfig ledSequence : ledSequences) {
            setLEDSequence(ledSequence);
        }
    }

    private void setLEDSequence(LEDConfig ledSequence) {
        this.candle.setLEDs(
                ledSequence.red,
                ledSequence.green,
                ledSequence.blue,
                ledSequence.white,
                ledSequence.startIndex,
                ledSequence.count);
    }

    public static Animation getColorFlow(int red, int green, int blue, int white, double speed, int numLed,
            Direction direction, int ledOffset) {
        return new ColorFlowAnimation(red, green, blue, white, speed, numLed, direction, ledOffset);
    }

    public static Animation getFire(double brightness, double speeds, int numLed, double sparkling, double cooling,
            boolean reverseDirection, int ledOffset) {
        return new FireAnimation(brightness, speeds, numLed, sparkling, cooling, reverseDirection, ledOffset);
    }

    public static Animation getLarson(int red, int green, int blue, int white, double speed, int numLed,
            BounceMode bounceMode, int size, int ledOffset) {
        return new LarsonAnimation(red, green, blue, white, speed, numLed, bounceMode, size, ledOffset);
    }

    public static Animation getRainbow(double brightness, double speed, int numLed, boolean reverseDirection,
            int ledOffset) {
        return new RainbowAnimation(brightness, speed, numLed, reverseDirection, ledOffset);
    }

    public static Animation getRgbFade(double brightness, double speed, int numLed, int ledOffset) {
        return new RgbFadeAnimation(brightness, speed, numLed, ledOffset);
    }

    public static Animation getSingleFade(int red, int green, int blue, int white, double speed, int numLed,
            int ledOffset) {
        return new SingleFadeAnimation(red, green, blue, white, speed, numLed, ledOffset);
    }

    public static Animation getStrobe(int red, int green, int blue, int white, double speed, int numLed,
            int ledOffset) {
        return new StrobeAnimation(red, green, blue, white, speed, numLed, ledOffset);
    }

    public static Animation getTwinkle(int red, int green, int blue, int white, double speed, int numLed,
            TwinklePercent divider, int ledOffset) {
        return new TwinkleAnimation(red, green, blue, white, speed, numLed, divider, ledOffset);
    }

    public static Animation getTwinkleOff(int red, int green, int blue, int white, double speed, int numLed,
            TwinkleOffPercent divider, int ledOffset) {
        return new TwinkleOffAnimation(red, green, blue, white, speed, numLed, divider, ledOffset);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("CurrentAnimation", this.currentAnimationName);
        if(currentLEDConfig != null){
            this.setLEDSequence(currentLEDConfig);
        }
    }

    public CommandBase getAnimationCommand(
            String animationName,
            Animation animation) {
        return Commands.runOnce(
                () -> setAnimations(animationName, animation),
                this);
    }

    public CommandBase getSetLEDCommand(
            int r, int g, int b) {
        return Commands.runOnce(
                () -> setLEDSequence(
                        this.currentLEDConfig = new LEDConfig(r, g, b, 0, 0, 300)),
                this);
    }

    public CommandBase getLEDOffCommand() {
        return Commands.runOnce(
                () -> animateOff(),
                this);
    }

    public CommandBase getDanceParty(){
        Runnable danceParty = new Runnable() {

            @Override
            public void run() {
                Animation fire = getFire(1, 1, 400, 1, 1, false, 0);
                setAnimation(fire);                
            }            
        };

        return Commands.runOnce(danceParty,this);
    }

}
