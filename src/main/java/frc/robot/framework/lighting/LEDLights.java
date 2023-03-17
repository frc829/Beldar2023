// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting;

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
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import frc.robot.framework.lighting.animations.multiColor.Fire;
import frc.robot.framework.lighting.animations.multiColor.RGBFade;
import frc.robot.framework.lighting.animations.multiColor.Rainbow;
import frc.robot.framework.lighting.animations.singleColor.ColorFlow;
import frc.robot.framework.lighting.animations.singleColor.Larson;
import frc.robot.framework.lighting.animations.singleColor.SingleColorForever;
import frc.robot.framework.lighting.animations.singleColor.SingleFade;
import frc.robot.framework.lighting.animations.singleColor.Strobe;
import frc.robot.framework.lighting.animations.singleColor.TwinkleOnOrOff;
import frc.robot.framework.lighting.animations.singleColor.Larson.Bounce;

/** Add your docs here. */
public interface LEDLights {

    public void setLEDAnimation(SingleColorForever singleColorForever);

    public void setLEDAnimation(SingleFade singleFade);

    public void setLEDAnimation(ColorFlow colorFlow);

    public void setLEDAnimation(Larson larson);

    public void setLEDAnimation(Strobe strobe);

    public void setLEDAnimation(TwinkleOnOrOff twinkleOnOrOff);

    public void setLEDAnimation(RGBFade rgbFade);

    public void setLEDAnimation(Rainbow rainbow);

    public void setLEDAnimation(Fire fire);

    public void clearLEDAnimations();

    public static LEDLights create(CANdle caNdle) {

        return new LEDLights() {

            @Override
            public void setLEDAnimation(SingleColorForever singleColorForever) {
                SingleFadeAnimation singleFadeAnimation = new SingleFadeAnimation(
                        singleColorForever.getRed(),
                        singleColorForever.getGreen(),
                        singleColorForever.getBlue(),
                        singleColorForever.getWhite(),
                        0,
                        singleColorForever.getNumLed(),
                        singleColorForever.getLedStart());
                caNdle.animate(singleFadeAnimation);
            }

            @Override
            public void setLEDAnimation(SingleFade singleFade) {
                SingleFadeAnimation singleFadeAnimation = new SingleFadeAnimation(
                        singleFade.getRed(),
                        singleFade.getGreen(),
                        singleFade.getBlue(),
                        singleFade.getWhite(),
                        singleFade.getSpeed(),
                        singleFade.getNumLed(),
                        singleFade.getLedStart());
                caNdle.animate(singleFadeAnimation);
            }

            @Override
            public void setLEDAnimation(ColorFlow colorFlow) {
                ColorFlowAnimation colorFlowAnimation = new ColorFlowAnimation(
                        colorFlow.getRed(),
                        colorFlow.getGreen(),
                        colorFlow.getBlue(),
                        colorFlow.getWhite(),
                        colorFlow.getSpeed(),
                        colorFlow.getNumLed(),
                        colorFlow.getIsReversed() ? Direction.Backward : Direction.Forward,
                        colorFlow.getLedStart());
                caNdle.animate(colorFlowAnimation);
            }

            @Override
            public void setLEDAnimation(Larson larson) {
                BounceMode bounceMode = BounceMode.Front;

                if (larson.getBounce() == Bounce.Center) {
                    bounceMode = BounceMode.Center;
                } else if (larson.getBounce() == Bounce.Back) {
                    bounceMode = BounceMode.Back;
                }

                LarsonAnimation larsonAnimation = new LarsonAnimation(
                        larson.getRed(),
                        larson.getGreen(),
                        larson.getBlue(),
                        larson.getWhite(),
                        larson.getSpeed(),
                        larson.getNumLed(),
                        bounceMode,
                        larson.getSize(),
                        larson.getLedStart());
                caNdle.animate(larsonAnimation);
            }

            @Override
            public void setLEDAnimation(Strobe strobe) {
                StrobeAnimation strobeAnimation = new StrobeAnimation(
                        strobe.getRed(),
                        strobe.getGreen(),
                        strobe.getBlue(),
                        strobe.getWhite(),
                        strobe.getSpeed(),
                        strobe.getNumLed(),
                        strobe.getLedStart());
                caNdle.animate(strobeAnimation);

            }

            @Override
            public void setLEDAnimation(TwinkleOnOrOff twinkleOnOrOff) {
                if (twinkleOnOrOff.isTwinkleOn) {
                    TwinklePercent twinklePercent = TwinklePercent.Percent100;
                    if (twinkleOnOrOff.getDivider() == 1) {
                        twinklePercent = TwinklePercent.Percent88;
                    } else if (twinkleOnOrOff.getDivider() == 2) {
                        twinklePercent = TwinklePercent.Percent76;
                    } else if (twinkleOnOrOff.getDivider() == 3) {
                        twinklePercent = TwinklePercent.Percent64;
                    } else if (twinkleOnOrOff.getDivider() == 4) {
                        twinklePercent = TwinklePercent.Percent42;
                    } else if (twinkleOnOrOff.getDivider() == 5) {
                        twinklePercent = TwinklePercent.Percent30;
                    } else if (twinkleOnOrOff.getDivider() == 6) {
                        twinklePercent = TwinklePercent.Percent18;
                    } else if (twinkleOnOrOff.getDivider() == 7) {
                        twinklePercent = TwinklePercent.Percent6;
                    }
                    TwinkleAnimation twinkleAnimation = new TwinkleAnimation(
                            twinkleOnOrOff.getRed(),
                            twinkleOnOrOff.getGreen(),
                            twinkleOnOrOff.getBlue(),
                            twinkleOnOrOff.getWhite(),
                            twinkleOnOrOff.getSpeed(),
                            twinkleOnOrOff.getNumLed(),
                            twinklePercent,
                            twinkleOnOrOff.getLedStart());
                    caNdle.animate(twinkleAnimation);
                } else {
                    TwinkleOffPercent twinkleOffPercent = TwinkleOffPercent.Percent100;
                    if (twinkleOnOrOff.getDivider() == 1) {
                        twinkleOffPercent = TwinkleOffPercent.Percent88;
                    } else if (twinkleOnOrOff.getDivider() == 2) {
                        twinkleOffPercent = TwinkleOffPercent.Percent76;
                    } else if (twinkleOnOrOff.getDivider() == 3) {
                        twinkleOffPercent = TwinkleOffPercent.Percent64;
                    } else if (twinkleOnOrOff.getDivider() == 4) {
                        twinkleOffPercent = TwinkleOffPercent.Percent42;
                    } else if (twinkleOnOrOff.getDivider() == 5) {
                        twinkleOffPercent = TwinkleOffPercent.Percent30;
                    } else if (twinkleOnOrOff.getDivider() == 6) {
                        twinkleOffPercent = TwinkleOffPercent.Percent18;
                    } else if (twinkleOnOrOff.getDivider() == 7) {
                        twinkleOffPercent = TwinkleOffPercent.Percent6;
                    }
                    TwinkleOffAnimation twinkleOffAnimation = new TwinkleOffAnimation(
                            twinkleOnOrOff.getRed(),
                            twinkleOnOrOff.getGreen(),
                            twinkleOnOrOff.getBlue(),
                            twinkleOnOrOff.getWhite(),
                            twinkleOnOrOff.getSpeed(),
                            twinkleOnOrOff.getNumLed(),
                            twinkleOffPercent,
                            twinkleOnOrOff.getLedStart());
                    caNdle.animate(twinkleOffAnimation);
                }
            }

            @Override
            public void setLEDAnimation(RGBFade rgbFade) {
                RgbFadeAnimation rgbFadeAnimation = new RgbFadeAnimation(
                        rgbFade.getBrightness(),
                        rgbFade.getSpeed(),
                        rgbFade.getNumLed(),
                        rgbFade.getLedStart());
                caNdle.animate(rgbFadeAnimation);
            }

            @Override
            public void setLEDAnimation(Rainbow rainbow) {
                RainbowAnimation rainbowAnimation = new RainbowAnimation(
                        rainbow.getBrightness(),
                        rainbow.getSpeed(),
                        rainbow.getNumLed(),
                        rainbow.getIsReversed(),
                        rainbow.getLedStart());
                caNdle.animate(rainbowAnimation);

            }

            @Override
            public void setLEDAnimation(Fire fire) {
                FireAnimation fireAnimation = new FireAnimation(
                        fire.getBrightness(),
                        fire.getSpeed(),
                        fire.getNumLed(),
                        fire.getSparkling(),
                        fire.getCooling(),
                        fire.getIsReversed(),
                        fire.getLedStart());
                caNdle.animate(fireAnimation);
            }

            @Override
            public void clearLEDAnimations() {
                caNdle.animate(null);
            }
        };
    }
}
