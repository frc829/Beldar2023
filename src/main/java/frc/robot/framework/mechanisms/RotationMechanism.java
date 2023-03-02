// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface RotationMechanism {

    public Rotation2d reduceMotorRotations(Rotation2d motorEncoderMeasurement);

    public class REVMAXPlanetary {

        public enum GearStage {
            ThreeToOne(3.0), FourToOne(4.0), FiveToOne(5.0);

            private double value;

            private GearStage(double value) {
                this.value = value;
            }
        }
    }

    public static RotationMechanism create(
            REVMAXPlanetary.GearStage firstStage,
            REVMAXPlanetary.GearStage... remainingStages) {



        return new RotationMechanism() {

            @Override
            public Rotation2d reduceMotorRotations(Rotation2d motorEncoderMeasurement) {
                double reduction = firstStage.value;
                for(REVMAXPlanetary.GearStage gearStage : remainingStages){
                    reduction *= gearStage.value;
                }
                return motorEncoderMeasurement.times(reduction);
            }

        };
    }
}
