package frc.robot.framework.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;

public interface MotorController {

    public void setVelocity(Rotation2d rotationsPerSecond);

    public void stop();

    public static MotorController create(CANSparkMax sparkMax) {

        SparkMaxPIDController sparkMaxPIDController = sparkMax.getPIDController();

        return new MotorController() {

            @Override
            public void setVelocity(Rotation2d rotationsPerSecond) {
                Rotation2d rotationsPerMinute = rotationsPerSecond.times(60.0);
                double rotationsPerMinuteValue = rotationsPerMinute.getRotations();
                sparkMaxPIDController.setReference(rotationsPerMinuteValue, ControlType.kVelocity);
            }

            @Override
            public void stop() {
                sparkMaxPIDController.setReference(0.0, ControlType.kVoltage);
            }
        };
    }
}
