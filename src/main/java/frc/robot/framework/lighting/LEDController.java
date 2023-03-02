package frc.robot.framework.lighting;

import com.ctre.phoenix.led.CANdle;

import frc.robot.framework.lighting.AnimationConfig.ColorFlow;
import frc.robot.framework.lighting.AnimationConfig.Fire;
import frc.robot.framework.lighting.AnimationConfig.Larson;
import frc.robot.framework.lighting.AnimationConfig.RGBFade;
import frc.robot.framework.lighting.AnimationConfig.Rainbow;
import frc.robot.framework.lighting.AnimationConfig.SingleFade;
import frc.robot.framework.lighting.AnimationConfig.Strobe;
import frc.robot.framework.lighting.AnimationConfig.Twinkle;
import frc.robot.framework.lighting.AnimationConfig.TwinkleOff;

public interface LEDController {
    
    public void setBrightness(double percent);

    public void modulateBatteryOutput(double percent);

    public void setLEDS(LEDConfig... ledConfigs);

    public void LEDSOff();

    public void setColorFlow(AnimationConfig.ColorFlow colorFlowConfig);

    public void setFire(AnimationConfig.Fire fireConfig);

    public void setLarson(AnimationConfig.Larson larsonConfig);

    public void setRainbow(AnimationConfig.Rainbow rainbowConfig);

    public void setRGBFade(AnimationConfig.RGBFade rgbFadeConfig);

    public void setSingleFade(AnimationConfig.SingleFade singleFadeConfig);

    public void setStrobe(AnimationConfig.Strobe strobeConfig);

    public void setTwinkle(AnimationConfig.Twinkle twinkleConfig);

    public void setTwinkleOff(AnimationConfig.TwinkleOff twinkleOffConfig);

    public static LEDController create(CANdle candle){
        return new LEDController() {

            @Override
            public void setBrightness(double percent) {
                candle.configBrightnessScalar(percent, 0);             
            }

            @Override
            public void modulateBatteryOutput(double percent) {
                candle.modulateVBatOutput(percent);          
            }  

            @Override
            public void setLEDS(LEDConfig... ledConfigs) {
                candle.animate(null);
                for (LEDConfig ledConfig : ledConfigs) {
                    candle.setLEDs(
                        ledConfig.red, 
                        ledConfig.green, 
                        ledConfig.blue, 
                        ledConfig.white, 
                        ledConfig.startIndex, 
                        ledConfig.count);
                }            
            }

            @Override
            public void LEDSOff() {
                // TODO Auto-generated method stub                
            }

            @Override
            public void setColorFlow(ColorFlow colorFlowConfig) {
                // TODO Auto-generated method stub                
            }

            @Override
            public void setFire(Fire fireConfig) {
                // TODO Auto-generated method stub                
            }

            @Override
            public void setLarson(Larson larsonConfig) {
                // TODO Auto-generated method stub                
            }

            @Override
            public void setRainbow(Rainbow rainbowConfig) {
                // TODO Auto-generated method stub                
            }

            @Override
            public void setRGBFade(RGBFade rgbFadeConfig) {
                // TODO Auto-generated method stub                
            }

            @Override
            public void setSingleFade(SingleFade singleFadeConfig) {
                // TODO Auto-generated method stub                
            }

            @Override
            public void setStrobe(Strobe strobeConfig) {
                // TODO Auto-generated method stub                
            }

            @Override
            public void setTwinkle(Twinkle twinkleConfig) {
                // TODO Auto-generated method stub                
            }

            @Override
            public void setTwinkleOff(TwinkleOff twinkleOffConfig) {
                // TODO Auto-generated method stub                
            }

          
        };
    }


}
