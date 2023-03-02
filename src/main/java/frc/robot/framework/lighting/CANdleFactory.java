// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.lighting;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

/** Add your docs here. */
public abstract class CANdleFactory {

    public static CANdle create(
            int deviceId,
            String canbus,
            int ledCount,
            LEDStripType type,
            double brightness,
            boolean disableWhenLOS,
            boolean disableWhenRunning,
            VBatOutputMode vBatOutputMode,
            double dutyCyclePrcnt) {

        CANdle candle = new CANdle(deviceId, canbus);
        candle.configLEDType(type, 0);
        candle.configBrightnessScalar(brightness, 0);
        candle.configLOSBehavior(disableWhenLOS, 0);
        candle.configStatusLedState(disableWhenRunning, 0);
        candle.configVBatOutput(vBatOutputMode);
        candle.modulateVBatOutput(dutyCyclePrcnt);

        return candle;
    }

}
