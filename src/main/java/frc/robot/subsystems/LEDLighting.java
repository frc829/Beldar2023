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
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.framework.lighting.LEDConfig;

/** Add your docs here. */
public class LEDLighting extends SubsystemBase {

    private final CANdle candle;
    private Animation currentAnimation;
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
        this.currentAnimation = null;

    }

    public void changeBrightness(double percent) {
        this.candle.configBrightnessScalar(percent, 0);
    }

    public void modulateBatteryOutput(double percent) {
        this.candle.modulateVBatOutput(percent);
    }

    public void setLEDS() {

        if (currentLEDConfig != null) {
            this.currentAnimation = null;
            this.candle.setLEDs(
                    currentLEDConfig.red,
                    currentLEDConfig.green,
                    currentLEDConfig.blue,
                    currentLEDConfig.white,
                    currentLEDConfig.startIndex,
                    currentLEDConfig.count);
        } else {
            this.candle.setLEDs(0, 0, 0, 0, 0, 400);
        }
    }

    public void setAnimation() {
        this.candle.animate(currentAnimation);
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
        if (this.currentAnimation == null) {
            SmartDashboard.putString("LED Lighting Current Animation", "NONE");
        } else {
            SmartDashboard.putString("LED Lighting Current Animation", this.currentAnimation.toString());
        }
        if (this.currentLEDConfig == null) {
            SmartDashboard.putString("LED Lighting Current COLOR", "NONE");
        } else {
            SmartDashboard.putString("LED Lighting Current COLOR", this.currentLEDConfig.toString());
        }

        this.setLEDS();
        this.setAnimation();
    }

    public CommandBase createControlCommand(
            Color8Bit color) {

        CommandBase setLEDCommand = new CommandBase() {
            @Override
            public void initialize() {
                currentAnimation = null;
                candle.animate(currentAnimation);
                SmartDashboard.putString("LED Lighting Current Command", color.red + ":" + color.green + ":" + color.blue);
                currentLEDConfig = new LEDConfig(color, 0, 400);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };

        setLEDCommand.addRequirements(this);
        return setLEDCommand;
    }

    public CommandBase getDanceParty() {

        CommandBase dancePartyCommand = new CommandBase() {

            @Override
            public void initialize() {
                SmartDashboard.putString("LED Lighting Current Command", "Dance Party");
                Animation dance = getRainbow(1, 1, 400, false, 0);
                currentAnimation = dance;
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };

        dancePartyCommand.addRequirements(this);
        return dancePartyCommand;
    }

    public CommandBase getDanceParty2() {

        CommandBase dancePartyCommand = new CommandBase() {

            @Override
            public void initialize() {
                SmartDashboard.putString("LED Lighting Current Command", "Dance Party");
                currentLEDConfig = null;
                Animation dance = getRainbow(1, 1, 400, false, 0);
                currentAnimation = dance;
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };

        dancePartyCommand.addRequirements(this);
        return dancePartyCommand;
    }

}
