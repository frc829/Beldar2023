package frc.robot.framework.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public interface Motor {

    public Rotation2d getPositionRotations();

    public Rotation2d getVelocityRotationPerSecond();

    public void setEncoderPosition(Rotation2d rotation2d);

    public void setVelocityRotationsPerSecond(Rotation2d velocityRotation2dPerSecond);

    public double getVoltage();

    public DCMotor getDCMotor();

    public void stop();

    public static Motor create(
        CANSparkMax sparkMax, 
        REVMotor sparkMaxBrand) {
              
        return new Motor() {

            @Override
            public Rotation2d getPositionRotations() {
                double rotations = sparkMax.getEncoder(Type.kHallSensor, 42).getPosition();
                return Rotation2d.fromRotations(rotations);
            }

            @Override
            public Rotation2d getVelocityRotationPerSecond() {
                double rotationsPerMinute = sparkMax.getEncoder(Type.kHallSensor, 42).getVelocity();
                double rotationsPerSecond = rotationsPerMinute / 60.0;
                return Rotation2d.fromRotations(rotationsPerSecond);
            }

            @Override
            public void setEncoderPosition(Rotation2d rotation2d) {
                double rotations = rotation2d.getRotations();
                sparkMax.getEncoder(Type.kHallSensor, 42).setPosition(rotations);
            }

            @Override
            public void setVelocityRotationsPerSecond(Rotation2d velocityRotation2dPerSecond) {
                double velocityRotationsPerSecond = velocityRotation2dPerSecond.getRotations();
                double velocityRotationsPerMinute = velocityRotationsPerSecond * 60.0;
                sparkMax.getPIDController().setReference(velocityRotationsPerMinute, ControlType.kVelocity);     
            }

            @Override
            public void stop() {
                sparkMax.getPIDController().setReference(0.0, ControlType.kVoltage);
            }

            @Override
            public double getVoltage() {
                return sparkMax.getBusVoltage() * sparkMax.getAppliedOutput();
            }

            @Override
            public DCMotor getDCMotor() {
                if(sparkMaxBrand == REVMotor.NEO550){
                    return DCMotor.getNeo550(1);
                }
                else{
                    return DCMotor.getNEO(1);
                }
            }
        };
    }

    public enum REVMotor{
        NEO,
        NEO550
    }
}
