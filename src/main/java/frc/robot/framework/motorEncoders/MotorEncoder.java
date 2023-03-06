package frc.robot.framework.motorEncoders;

import com.revrobotics.RelativeEncoder;

public interface MotorEncoder {
    public double getPositionRotations();

    public double getVelocityRotationsPerSecond();

    public void setPositionRotations(double rotations);

    public static MotorEncoder create(RelativeEncoder relativeEncoder) {
        return new MotorEncoder() {

            @Override
            public double getPositionRotations() {
                return relativeEncoder.getPosition();
            }

            @Override
            public double getVelocityRotationsPerSecond() {
                return relativeEncoder.getVelocity() / 60.0;
            }

            @Override
            public void setPositionRotations(double rotations) {
                relativeEncoder.setPosition(rotations);
            }
        };
    }
}
