// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public interface ManualChassisSpeedControl {

    public ChassisSpeeds getFieldCentricSpeeds(Rotation2d robotAngle);

    public ChassisSpeeds getRobotCentricSpeeds(Rotation2d robotAngle);

    public static ManualChassisSpeedControl getManualChassisSpeedControl(
            ManualSpeedControl fieldCentricForwardBackControl,
            ManualSpeedControl fieldCentricLeftRightControl,
            ManualSpeedControl robotCentricForwardBackControl,
            ManualSpeedControl robotCentricLeftRightControl,
            ManualSpeedControl robotRotationControl) {

        return new ManualChassisSpeedControl() {

            @Override
            public ChassisSpeeds getFieldCentricSpeeds(Rotation2d robotAngle) {
                return getTranslationSpeeds(
                        fieldCentricForwardBackControl,
                        fieldCentricLeftRightControl,
                        robotCentricForwardBackControl,
                        robotCentricLeftRightControl,
                        robotRotationControl,
                        robotAngle,
                        SpeedFrameOfReference.Field);
            }

            @Override
            public ChassisSpeeds getRobotCentricSpeeds(Rotation2d robotAngle) {
                return getTranslationSpeeds(
                        fieldCentricForwardBackControl,
                        fieldCentricLeftRightControl,
                        robotCentricForwardBackControl,
                        robotCentricLeftRightControl,
                        robotRotationControl,
                        robotAngle,
                        SpeedFrameOfReference.Robot);
            }
        };
    }

    private static ChassisSpeeds getTranslationSpeeds(
            ManualSpeedControl fieldCentricForwardBackControl,
            ManualSpeedControl fieldCentricLeftRightControl,
            ManualSpeedControl robotCentricForwardBackControl,
            ManualSpeedControl robotCentricLeftRightControl,
            ManualSpeedControl robotRotationControl,
            Rotation2d robotAngle,
            SpeedFrameOfReference speedFrameOfReference) {

        Translation2d fieldCentricTranslationSpeeds = new Translation2d(
                fieldCentricForwardBackControl.getManualSpeed(),
                fieldCentricLeftRightControl.getManualSpeed());

        Translation2d robotCentricTranslationSpeeds = new Translation2d(
                robotCentricForwardBackControl.getManualSpeed(),
                robotCentricLeftRightControl.getManualSpeed());

        if (speedFrameOfReference == SpeedFrameOfReference.Field) {
            if (fieldCentricTranslationSpeeds.getNorm() > 0) {
                return new ChassisSpeeds(
                        fieldCentricForwardBackControl.getManualSpeed(),
                        fieldCentricLeftRightControl.getManualSpeed(),
                        robotRotationControl.getManualSpeed());
            } else if (robotCentricTranslationSpeeds.getNorm() == 0) {
                return new ChassisSpeeds(0, 0, robotRotationControl.getManualSpeed());
            } else {
                return ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                                robotCentricForwardBackControl.getManualSpeed(),
                                robotCentricLeftRightControl.getManualSpeed(),
                                robotRotationControl.getManualSpeed()),
                        robotAngle.unaryMinus());
            }
        } else {
            if (fieldCentricTranslationSpeeds.getNorm() > 0) {
                return ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                                fieldCentricForwardBackControl.getManualSpeed(),
                                fieldCentricLeftRightControl.getManualSpeed(),
                                robotRotationControl.getManualSpeed()),
                        robotAngle);
            } else if (fieldCentricTranslationSpeeds.getNorm() == 0 && robotCentricTranslationSpeeds.getNorm() > 0) {
                return new ChassisSpeeds(
                        robotCentricForwardBackControl.getManualSpeed(),
                        robotCentricLeftRightControl.getManualSpeed(),
                        robotRotationControl.getManualSpeed());
            } else {
                return new ChassisSpeeds(0, 0, robotRotationControl.getManualSpeed());
            }
        }
    }

    public enum SpeedFrameOfReference {
        Field,
        Robot
    }
}
