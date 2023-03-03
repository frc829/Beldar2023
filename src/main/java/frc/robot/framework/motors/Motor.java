package frc.robot.framework.motors;

import edu.wpi.first.math.geometry.Rotation2d;

public class Motor {   

    private final MotorController motorController;
    private final MotorEncoder motorEncoder;

    public Motor(MotorController motorController, MotorEncoder motorEncoder){
        this.motorController = motorController;
        this.motorEncoder = motorEncoder;
    }

    public Rotation2d getPosition(){
        return motorEncoder.getPosition();
    }
    
    public Rotation2d getVelocity(){
        return motorEncoder.getVelocityPerSecond();
    }

    public void setPosition(Rotation2d rotations){
        motorEncoder.setPosition(rotations);
    }

    public void setVelocity(Rotation2d rotationsPerSecond){
        motorController.setVelocity(rotationsPerSecond);
    }

    public void stop(){
        motorController.stop();
    }
}
