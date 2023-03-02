// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting;

/** Add your docs here. */
public class AnimationConfig {
    public enum BounceType {
        Front,
        Center,
        Back
    }

    public class ColorFlow {
        public final int red;
        public final int green;
        public final int blue;
        public final int white;
        public final double speed;
        public final int numLED;
        public final boolean isReversed;
        public final int ledOffset;

        public ColorFlow(
                int red,
                int green,
                int blue,
                int white,
                double speed,
                int numLED,
                boolean isReversed,
                int ledOffset) {
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.white = white;
            this.speed = speed;
            this.numLED = numLED;
            this.isReversed = isReversed;
            this.ledOffset = ledOffset;
        }
    }

    public class Fire {
        public final double brightness;
        public final double speeds;
        public final int numLed;
        public final double sparkling;
        public final double cooling;
        public final boolean reverseDirection;
        public final int ledOffset;

        public Fire(
                double brightness,
                double speeds,
                int numLed,
                double sparkling,
                double cooling,
                boolean reverseDirection,
                int ledOffset) {
            this.brightness = brightness;
            this.speeds = speeds;
            this.numLed = numLed;
            this.sparkling = sparkling;
            this.cooling = cooling;
            this.reverseDirection = reverseDirection;
            this.ledOffset = ledOffset;
        }
    }

    public class Larson {
        public final int red;
        public final int green;
        public final int blue;
        public final int white;
        public final double speed;
        public final int numLed;
        public final BounceType bounceType;
        public final int size;
        public final int ledOffset;

        public Larson(
                int red,
                int green,
                int blue,
                int white,
                double speed,
                int numLed,
                BounceType bounceType,
                int size,
                int ledOffset) {
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.white = white;
            this.speed = speed;
            this.numLed = numLed;
            this.size = size;
            this.bounceType = bounceType;
            this.ledOffset = ledOffset;
        }
    }

    public class Rainbow {
        public final double brightness;
        public final double speeds;
        public final int numLed;
        public final boolean reverseDirection;
        public final int ledOffset;

        public Rainbow(
                double brightness,
                double speeds,
                int numLed,
                boolean reverseDirection,
                int ledOffset) {
            this.brightness = brightness;
            this.speeds = speeds;
            this.numLed = numLed;
            this.reverseDirection = reverseDirection;
            this.ledOffset = ledOffset;
        }
    }

    public class RGBFade {
        public final double brightness;
        public final double speeds;
        public final int numLed;
        public final int ledOffset;

        public RGBFade(
                double brightness,
                double speeds,
                int numLed,
                int ledOffset) {
            this.brightness = brightness;
            this.speeds = speeds;
            this.numLed = numLed;
            this.ledOffset = ledOffset;
        }
    }

    public class SingleFade {
        public final int red;
        public final int green;
        public final int blue;
        public final int white;
        public final int numLed;
        public final double speed;
        public final int ledOffset;

        public SingleFade(
                int red,
                int green,
                int blue,
                int white,
                double speed,
                int numLed,
                int ledOffset) {
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.white = white;
            this.speed = speed;
            this.numLed = numLed;
            this.ledOffset = ledOffset;
        }
    }

    public class Strobe {
        public final int red;
        public final int green;
        public final int blue;
        public final int white;
        public final int numLed;
        public final double speed;
        public final int ledOffset;

        public Strobe(
                int red,
                int green,
                int blue,
                int white,
                double speed,
                int numLed,
                int ledOffset) {
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.white = white;
            this.speed = speed;
            this.numLed = numLed;
            this.ledOffset = ledOffset;
        }
    }

    public class Twinkle {
        public final int red;
        public final int green;
        public final int blue;
        public final int white;
        public final int numLed;
        public final double speed;
        public final int twinklePercent;
        public final int ledOffset;

        public Twinkle(
                int red,
                int green,
                int blue,
                int white,
                double speed,
                int numLed,
                int twinklePercent,
                int ledOffset) {
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.white = white;
            this.speed = speed;
            this.numLed = numLed;
            this.twinklePercent = twinklePercent;
            this.ledOffset = ledOffset;
        }
    }

    public class TwinkleOff {
        public final int red;
        public final int green;
        public final int blue;
        public final int white;
        public final int numLed;
        public final double speed;
        public final int twinkleOffPercent;
        public final int ledOffset;

        public TwinkleOff(
                int red,
                int green,
                int blue,
                int white,
                double speed,
                int numLed,
                int twinkleOffPercent,
                int ledOffset) {
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.white = white;
            this.speed = speed;
            this.numLed = numLed;
            this.twinkleOffPercent = twinkleOffPercent;
            this.ledOffset = ledOffset;
        }
    }

}
