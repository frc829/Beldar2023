// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.swerveDriveComponents;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.framework.mechanismsAdvanced.SwerveModule;

/** Add your docs here. */
public class SwerveDriveTelemetry {

    private final Field2d theField;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private Pose2d initialPosition;

    public SwerveDriveTelemetry(
            Field2d theField,
            Pose2d initialPosition,
            SwerveDriveKinematics swerveDriveKinematics,
            SwerveModule... swerveModules) {
        this.theField = theField;
        this.initialPosition = initialPosition;

        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];

        for(int i = 0; i < swerveModules.length; i++){
            swerveModulePositions[i] = swerveModules[i].getSwerveModulePosition();
        }
       
        this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            swerveDriveKinematics, 
            new Rotation2d(), 
            swerveModulePositions, 
            initialPosition);

        this.theField.setRobotPose(initialPosition);
        SmartDashboard.putData(theField);
    }

    public Pose2d getCurrentInitialPosition() {
        return this.initialPosition;
    }

    public Pose2d getSwerveDrivePosition() {
        return this.swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void updateSwerveDrivePosition(
            Rotation2d robotAngle,
            SwerveModulePosition... swerveModulePositions) {

        

        this.swerveDrivePoseEstimator.updateWithTime(
            Timer.getFPGATimestamp(), 
            robotAngle, 
            swerveModulePositions);

        this.theField.setRobotPose(this.swerveDrivePoseEstimator.getEstimatedPosition());
    }

    public void resetSwerveDrivePosition(
            Pose2d newPose,
            Rotation2d gyroScopeAngle,
            SwerveModulePosition... swerveModulePositions) {

        this.initialPosition = newPose;

        this.swerveDrivePoseEstimator.resetPosition(
            gyroScopeAngle, 
            swerveModulePositions, 
            newPose);

        

    }

}
