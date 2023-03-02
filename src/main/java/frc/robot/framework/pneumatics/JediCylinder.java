// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;

/** Add your docs here. */
public interface JediCylinder {

    public JediCylinder.State getExtendedState();

    public void setExtendedState(JediCylinder.State jediCylinderState);

    public enum State {
        Extended,
        Retracted,
        Unknown
    }

    public static JediCylinder getSingleSolenoidBased(Solenoid solenoid) {

        SolenoidSim solenoidSim = new SolenoidSim(
            19, 
            PneumaticsModuleType.REVPH, 
            solenoid.getChannel());

        return new JediCylinder() {

            @Override
            public State getExtendedState() {

                if(RobotBase.isSimulation()){
                    return solenoidSim.getOutput() ? JediCylinder.State.Extended : JediCylinder.State.Retracted;
                }
                else{
                    return solenoid.get() ? JediCylinder.State.Extended : JediCylinder.State.Retracted;
                }


            }

            @Override
            public void setExtendedState(State jediCylinderState) {

                if(RobotBase.isSimulation()){
                    boolean isSolenoidOn = jediCylinderState == JediCylinder.State.Extended ? true : false;
                    solenoidSim.setOutput(isSolenoidOn);
                }
                else{
                    boolean isSolenoidOn = jediCylinderState == JediCylinder.State.Extended ? true : false;
                    solenoid.set(isSolenoidOn);
                }


            }
        };
    }

    public static JediCylinder getDoubleSolenoidBased(DoubleSolenoid doubleSolenoid) {

        DoubleSolenoidSim doubleSolenoidSim = new DoubleSolenoidSim(
            19, 
            PneumaticsModuleType.REVPH, 
            doubleSolenoid.getFwdChannel(), 
            doubleSolenoid.getRevChannel());

        return new JediCylinder() {

            @Override
            public State getExtendedState() {
                if(RobotBase.isSimulation()){
                    if (doubleSolenoidSim.get() == Value.kForward) {
                        return JediCylinder.State.Extended;
                    } else if (doubleSolenoidSim.get() == Value.kReverse) {
                        return JediCylinder.State.Retracted;
                    } else {
                        return JediCylinder.State.Unknown;
                    }
                }
                else{
                    if (doubleSolenoid.get() == Value.kForward) {
                        return JediCylinder.State.Extended;
                    } else if (doubleSolenoid.get() == Value.kReverse) {
                        return JediCylinder.State.Retracted;
                    } else {
                        return JediCylinder.State.Unknown;
                    }
                }

            }

            @Override
            public void setExtendedState(State jediCylinderState) {
                if(RobotBase.isSimulation()){
                    if (jediCylinderState == JediCylinder.State.Extended) {
                        doubleSolenoidSim.set(Value.kForward);
                    } else if (jediCylinderState == JediCylinder.State.Retracted) {
                        doubleSolenoidSim.set(Value.kReverse);
                    } else {
                        doubleSolenoidSim.set(Value.kOff);
                    }
                }
                else{
                    if (jediCylinderState == JediCylinder.State.Extended) {
                        doubleSolenoid.set(Value.kForward);
                    } else if (jediCylinderState == JediCylinder.State.Retracted) {
                        doubleSolenoid.set(Value.kReverse);
                    } else {
                        doubleSolenoid.set(Value.kOff);
                    }
                }

            }
        };
    }

}
