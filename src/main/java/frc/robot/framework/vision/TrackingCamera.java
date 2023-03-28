package frc.robot.framework.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.types.LimelightHelpers;

public interface TrackingCamera {

    public double[] getFieldPosition(Alliance alliance);

    public static TrackingCamera createFromLimeLight(String limelightName) {

        return new TrackingCamera() {

            @Override
            public double[] getFieldPosition(Alliance alliance) {
                double[] fieldPoseDoubleArray = new double[7];

                double[] blue = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
                double[] red = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[7]);
                fieldPoseDoubleArray = alliance == Alliance.Blue
                        ? blue
                        : red;

                fieldPoseDoubleArray = fieldPoseDoubleArray.length == 0 ? new double[7] : fieldPoseDoubleArray;

                fieldPoseDoubleArray[3] = fieldPoseDoubleArray[3] < 0 ? fieldPoseDoubleArray[3] + 360
                        : fieldPoseDoubleArray[3];
                fieldPoseDoubleArray[4] = fieldPoseDoubleArray[4] < 0 ? fieldPoseDoubleArray[4] + 360
                        : fieldPoseDoubleArray[4];
                fieldPoseDoubleArray[5] = fieldPoseDoubleArray[5] < 0 ? fieldPoseDoubleArray[5] + 360
                        : fieldPoseDoubleArray[5];

                return fieldPoseDoubleArray;
            }

        };
    }

}
