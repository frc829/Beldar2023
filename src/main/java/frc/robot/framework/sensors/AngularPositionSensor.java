package frc.robot.framework.sensors;

import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.framework.motors.Motor;
import frc.robot.framework.simulation.Simulator;

public interface AngularPositionSensor {

    /**
     * Returns the absolute angle measured by the sensor.
     * This angle is forced into the range 0-360 degrees, 0-2PI radians, 0-1 turns
     * 
     * @return absolute angle of the sensor
     */
    public Rotation2d getAbsoluteAngle();

    public static AngularPositionSensor create(
            WPI_CANCoder canCoder) {

        return new AngularPositionSensor() {

            @Override
            public Rotation2d getAbsoluteAngle() {
                if (canCoder.getMagnetFieldStrength() == MagnetFieldStrength.BadRange_RedLED ||
                        canCoder.getMagnetFieldStrength() == MagnetFieldStrength.Invalid_Unknown) {
                    return Rotation2d.fromDegrees(0.0);
                } else {
                    var position = canCoder.getAbsolutePosition();
                    return Rotation2d.fromDegrees(position);
                }
            }
        };
    }

    public static AngularPositionSensor getREVThroughBoreEncoder(
            int channel,
            double offSetDegrees,
            Motor motor,
            double motorToMechConversion) {
        DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(channel);
        dutyCycleEncoder.setDutyCycleRange(
                1.0 / 1025.0,
                1024.0 / 1025.0);

        if (RobotBase.isSimulation()) {
            Simulator.getInstance().addREVThroughBoreEncoder(
                    dutyCycleEncoder,
                    offSetDegrees,
                    motor,
                    motorToMechConversion);
        }

        return new AngularPositionSensor() {

            @Override
            public Rotation2d getAbsoluteAngle() {
                double absoluteRotations = dutyCycleEncoder.getAbsolutePosition();
                double absoluteDegrees = absoluteRotations * 360.0;
                double goodDegrees = absoluteDegrees - offSetDegrees;
                Rotation2d almostGoodangle = Rotation2d.fromDegrees(goodDegrees);

                double mechRotationsValue = almostGoodangle.getRotations();
                mechRotationsValue %= 1;
                mechRotationsValue = mechRotationsValue < 0 ? mechRotationsValue + 1 : mechRotationsValue;
                Rotation2d goodAngle = Rotation2d.fromRotations(mechRotationsValue);

                return goodAngle;
            }
        };
    }
}
