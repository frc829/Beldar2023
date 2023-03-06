package frc.robot.framework.motorControllers;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public interface MotorController {    
    public void setVelocityRPS(double velocityRPS);

    public void stop();

    public static MotorController create(SparkMaxPIDController sparkMaxPIDController){
        return new MotorController() {

            @Override
            public void setVelocityRPS(double velocityRPS) {
                double rpm = velocityRPS * 60.0;
                sparkMaxPIDController.setReference(rpm, ControlType.kVelocity);                
            }

            @Override
            public void stop() {
                sparkMaxPIDController.setReference(0.0, ControlType.kVelocity);                
            }            
        };
    }
}
