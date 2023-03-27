package frc.robot.framework.imus;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyroscope {

    public boolean isConnected();

    /**
     * Returns the yaw angle (Z-axis rotation) of the gyroscope as mounted on the
     * RIO
     * This angle is forced into the range 0-360 degrees, 0-2PI radians, 0-1 turns
     * 
     * @return yaw angle
     */
    public Rotation2d getYaw();

    /**
     * Returns the pitch angle (Y-axis rotation) of the gyroscope as mounted on the
     * RIO
     * This angle is forced into the range 0-360 degrees, 0-2PI radians, 0-1 turns
     * 
     * @return pitch angle
     */
    public Rotation2d getPitch();

    /**
     * Returns the roll angle (X-axis rotation) of the gyroscope as mounted on the
     * RIO
     * This angle is forced into the range 0-360 degrees, 0-2PI radians, 0-1 turns
     * 
     * @return roll angle
     */
    public Rotation2d getRoll();

    public Rotation2d getPitchRate();

    public static Gyroscope create(AHRS gyro) {

        return new Gyroscope() {

            @Override
            public Rotation2d getYaw() {
                double yawDegrees = -gyro.getYaw();
                yawDegrees %= 360;
                if (yawDegrees < 0) {
                    yawDegrees += 360;
                }
                return Rotation2d.fromDegrees(yawDegrees);
            }

            @Override
            public Rotation2d getPitch() {
                double pitchDegrees = gyro.getPitch();
                pitchDegrees %= 360;
                if (pitchDegrees < 0) {
                    pitchDegrees += 360;
                }
                return Rotation2d.fromDegrees(pitchDegrees);
            }

            @Override
            public Rotation2d getRoll() {
                double rollDegrees = gyro.getRoll();
                rollDegrees %= 360;
                if (rollDegrees < 0) {
                    rollDegrees += 360;
                }
                return Rotation2d.fromDegrees(rollDegrees);
            }

            @Override
            public boolean isConnected() {
                return gyro.isConnected();
            }

            @Override
            public Rotation2d getPitchRate() {
                return Rotation2d.fromDegrees(gyro.getRawGyroX());
            }
        };
    }


}
