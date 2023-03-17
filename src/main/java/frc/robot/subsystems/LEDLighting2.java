// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.framework.lighting.CANdleFactory;
import frc.robot.framework.lighting.LEDLights;
import frc.robot.framework.lighting.animations.Animation;
import frc.robot.framework.lighting.animations.multiColor.Fire;
import frc.robot.framework.lighting.animations.multiColor.RGBFade;
import frc.robot.framework.lighting.animations.multiColor.Rainbow;
import frc.robot.framework.lighting.animations.singleColor.ColorFlow;
import frc.robot.framework.lighting.animations.singleColor.Larson;
import frc.robot.framework.lighting.animations.singleColor.SingleColorForever;
import frc.robot.framework.lighting.animations.singleColor.SingleFade;
import frc.robot.framework.lighting.animations.singleColor.Strobe;
import frc.robot.framework.lighting.animations.singleColor.TwinkleOnOrOff;

public class LEDLighting2 extends SubsystemBase {

  private final LEDLights ledLights;

  public LEDLighting2() {

    CANdle caNdle = CANdleFactory.create(
        Constants.Robot.Arm.LEDS.Candle.deviceId,
        Constants.Robot.Arm.LEDS.Candle.canbus,
        Constants.Robot.Arm.LEDS.Candle.ledCount,
        Constants.Robot.Arm.LEDS.Candle.ledStripType,
        Constants.Robot.Arm.LEDS.Candle.brightness,
        Constants.Robot.Arm.LEDS.Candle.disableWhenLOS,
        Constants.Robot.Arm.LEDS.Candle.disableWhenRunning,
        Constants.Robot.Arm.LEDS.Candle.vBatOutputMode,
        Constants.Robot.Arm.LEDS.Candle.dutyCyclePrcnt);

    this.ledLights = LEDLights.create(caNdle);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Commands
  public CommandBase createAnimateCommand(Animation animation) {

    CommandBase animateCommand = new CommandBase() {
      @Override
      public void initialize() {
        if (animation instanceof SingleColorForever) {
          ledLights.setLEDAnimation((SingleColorForever) animation);
        }
        else if (animation instanceof ColorFlow) {
          ledLights.setLEDAnimation((ColorFlow) animation);
        }
        else if (animation instanceof Larson) {
          ledLights.setLEDAnimation((Larson) animation);
        }
        else if (animation instanceof SingleFade) {
          ledLights.setLEDAnimation((SingleFade) animation);
        }
        else if (animation instanceof Strobe) {
          ledLights.setLEDAnimation((Strobe) animation);
        }
        else if (animation instanceof Fire) {
          ledLights.setLEDAnimation((Fire) animation);
        }
        else if (animation instanceof Rainbow) {
          ledLights.setLEDAnimation((Rainbow) animation);
        }
        else if (animation instanceof RGBFade) {
          ledLights.setLEDAnimation((RGBFade) animation);
        }
        else if (animation instanceof TwinkleOnOrOff) {
          ledLights.setLEDAnimation((TwinkleOnOrOff) animation);
        }
        else{
          ledLights.clearLEDAnimations();
        }

      }

      @Override
      public boolean isFinished() {
        return true;
      }

    };

    animateCommand.addRequirements(this);

    return animateCommand;

  }

}
