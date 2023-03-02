package frc.robot.framework.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;

public interface MotorEncoder {

    public Rotation2d getPosition();

    public Rotation2d getVelocityPerSecond();

    public void setPosition(Rotation2d rotations);

    public static MotorEncoder createNEO550(CANSparkMax sparkMax) {
        return createNEO(sparkMax);
    }

    public static MotorEncoder createNEO(CANSparkMax sparkMax) {
        RelativeEncoder relativeEncoder = sparkMax.getEncoder(Type.kHallSensor, 42);
        return create(relativeEncoder);
    }

    private static MotorEncoder create(
            RelativeEncoder relativeEncoder) {

        return new MotorEncoder() {

            @Override
            public Rotation2d getPosition() {
                double rotationsValue = relativeEncoder.getPosition();
                return Rotation2d.fromRotations(rotationsValue);
            }

            @Override
            public Rotation2d getVelocityPerSecond() {
                double rotationsPerMinuteValue = relativeEncoder.getVelocity();
                double rotationsPerSecondValue = rotationsPerMinuteValue / 60.0;
                return Rotation2d.fromRotations(rotationsPerSecondValue);
            }

            @Override
            public void setPosition(Rotation2d rotations) {
                double rotationsValue = rotations.getRotations();
                relativeEncoder.setPosition(rotationsValue);
            }
        };
    }
}
