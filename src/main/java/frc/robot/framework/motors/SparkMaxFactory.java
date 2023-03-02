// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.framework.simulation.Simulator;

/** Add your docs here. */
public abstract class SparkMaxFactory {

    public enum REVMotor{
        NEO,
        NEO550
    }

    public static CANSparkMax create(
            int deviceId,
            REVMotor revMotor,
            boolean isInverted,
            IdleMode idleMode,
            double velocityKP,
            double velocityKI,
            double velocityKD,
            double velocityKF) {

        CANSparkMax canSparkMax = new CANSparkMax(deviceId, MotorType.kBrushless);
        canSparkMax.setInverted(isInverted);
        canSparkMax.setIdleMode(idleMode);
        canSparkMax.getPIDController().setP(velocityKP);
        canSparkMax.getPIDController().setI(velocityKI);
        canSparkMax.getPIDController().setD(velocityKD);
        canSparkMax.getPIDController().setFF(velocityKF);

        if(revMotor == REVMotor.NEO){
            canSparkMax.setSmartCurrentLimit(40);
            Simulator.getInstance().addSparkMax(canSparkMax, DCMotor.getNEO(deviceId));
        }
        else{
            canSparkMax.setSmartCurrentLimit(20);
            Simulator.getInstance().addSparkMax(canSparkMax, DCMotor.getNeo550(deviceId));
        }
        
        return canSparkMax;
    }
}
