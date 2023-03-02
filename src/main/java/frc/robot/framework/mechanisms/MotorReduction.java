// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface MotorReduction {

    public Rotation2d reduceMotorRotations(Rotation2d motorEncoderMeasurement);

    public Rotation2d expandMechanismRotations(Rotation2d mechanismMeasurement);

    public class REVMAXPlanetary {

        public enum GearStage {
            ThreeToOne(3.0), FourToOne(4.0), FiveToOne(5.0);

            private double value;

            private GearStage(double value) {
                this.value = value;
            }
        }
    }

    public static MotorReduction create(
            REVMAXPlanetary.GearStage firstStage,
            REVMAXPlanetary.GearStage... remainingStages) {

        return new MotorReduction() {

            @Override
            public Rotation2d reduceMotorRotations(Rotation2d motorEncoderMeasurement) {
                double reduction = firstStage.value;
                for (REVMAXPlanetary.GearStage gearStage : remainingStages) {
                    reduction *= gearStage.value;
                }
                return motorEncoderMeasurement.times(reduction);
            }

            @Override
            public Rotation2d expandMechanismRotations(Rotation2d mechanismMeasurement) {
                double reduction = firstStage.value;
                for (REVMAXPlanetary.GearStage gearStage : remainingStages) {
                    reduction *= gearStage.value;
                }
                return mechanismMeasurement.div(reduction);
            }

        };
    }
}
