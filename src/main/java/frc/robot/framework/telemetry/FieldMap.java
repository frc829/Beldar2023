// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.telemetry;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class FieldMap {

    private Field2d theField = new Field2d();

    public FieldMap(){
        SmartDashboard.putData("The Field", theField);
    }

    public void updateField(Pose2d robotPose){
        theField.setRobotPose(robotPose);
    }

    public void addTrajectoryToField(String name, List<Pose2d> poses){
        theField.getObject(name).setPoses(poses);
    }

    public void removeTrajectoryFromField(String name){
        theField.getObject(name).setPoses(new ArrayList<Pose2d>());
    }

}
