// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.imus;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.framework.mechanismsAdvanced.SwerveModule;
import frc.robot.framework.simulation.Simulator;

/** Add your docs here. */
public class NavXGyroscopeFactory {

    public static AHRS create(
        Port serial_port_id, 
        SwerveDriveKinematics swerveDriveKinematics,
        SwerveModule... swerveModules){
        AHRS navXMXP2 = new AHRS(serial_port_id);
        navXMXP2.reset();

        Simulator.getInstance().addSwerveModules(
            navXMXP2,
            swerveDriveKinematics,
            swerveModules);

        return navXMXP2;
    }
}
