package frc.robot.framework.motors;

public class Motor {   

    public final MotorController motorController;
    public final MotorEncoder motorEncoder;

    public Motor(MotorController motorController, MotorEncoder motorEncoder){
        this.motorController = motorController;
        this.motorEncoder = motorEncoder;
    }
}
