package frc.robot.framework.sensors;

import frc.robot.framework.sensors.WPI_TimeOfFlightFactory.WPI_TimeOfFlight;

public interface LinearPositionSensor {

    /**
     * Returns the distance measured by the sensor in meters.
     * This angle is forced into the range 0-360 degrees, 0-2PI radians, 0-1 turns
     * 
     * @return absolute angle of the sensor
     */
    public double getPosition();

    public boolean isRangeValid();

    public static LinearPositionSensor create(
            int sensorId,
            WPI_TimeOfFlight wpi_TimeOfFlight) {

        return new LinearPositionSensor() {

            @Override
            public double getPosition() {
                return wpi_TimeOfFlight.getRange() / 1000.0;
            }

            public boolean isRangeValid(){
                return wpi_TimeOfFlight.isRangeValid();
            }
        };
    }
}
