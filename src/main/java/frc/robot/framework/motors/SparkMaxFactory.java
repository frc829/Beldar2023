// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.framework.motors.Motor.REVMotor;

/** Add your docs here. */
public abstract class SparkMaxFactory {

    public static CANSparkMax create(
            int deviceId,
            REVMotor revMotor,
            boolean isInverted,
            IdleMode idleMode,
            double velocityKP,
            double velocityKI,
            double velocityKD,
            double velocityKF) {

        MotorType motorType = (revMotor != REVMotor.NEO && revMotor != REVMotor.NEO550)
                ? MotorType.kBrushed
                : MotorType.kBrushless;
        CANSparkMax canSparkMax = new CANSparkMax(deviceId, motorType);
        canSparkMax.setInverted(isInverted);
        canSparkMax.setIdleMode(idleMode);
        canSparkMax.getPIDController().setP(velocityKP);
        canSparkMax.getPIDController().setI(velocityKI);
        canSparkMax.getPIDController().setD(velocityKD);
        canSparkMax.getPIDController().setFF(velocityKF);

        setSmartCurrentLimit(canSparkMax, revMotor);
        addToSimulator(canSparkMax, revMotor);

        return canSparkMax;
    }

    private static void addToSimulator(CANSparkMax canSparkMax, REVMotor revMotor) {
        if(revMotor == REVMotor.NEO){
            REVPhysicsSim.getInstance().addSparkMax(canSparkMax, DCMotor.getNEO(1));
        }
        else{
            REVPhysicsSim.getInstance().addSparkMax(canSparkMax, DCMotor.getNeo550(1));
        }
    }

    private static void setSmartCurrentLimit(CANSparkMax canSparkMax, REVMotor sparkMaxBrand) {
        if(sparkMaxBrand == REVMotor.NEO){
            canSparkMax.setSmartCurrentLimit(40);
        }
        else{
            canSparkMax.setSmartCurrentLimit(20);
        }
    }

}
