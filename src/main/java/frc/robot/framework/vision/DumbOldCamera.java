// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.vision;

import edu.wpi.first.cameraserver.CameraServer;

/** Add your docs here. */
public class DumbOldCamera {

    public static void start() {

        // Creates UsbCamera and MjpegServer [1] and connects them
        CameraServer.startAutomaticCapture();
    }

}
