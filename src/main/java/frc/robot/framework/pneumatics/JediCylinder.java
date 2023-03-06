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

        return new JediCylinder() {

            @Override
            public State getExtendedState() {


                    return solenoid.get() ? JediCylinder.State.Extended : JediCylinder.State.Retracted;



            }

            @Override
            public void setExtendedState(State jediCylinderState) {



                    boolean isSolenoidOn = jediCylinderState == JediCylinder.State.Extended ? true : false;
                    solenoid.set(isSolenoidOn);



            }
        };
    }

    public static JediCylinder getDoubleSolenoidBased(DoubleSolenoid doubleSolenoid) {


        return new JediCylinder() {

            @Override
            public State getExtendedState() {

                    if (doubleSolenoid.get() == Value.kForward) {
                        return JediCylinder.State.Extended;
                    } else if (doubleSolenoid.get() == Value.kReverse) {
                        return JediCylinder.State.Retracted;
                    } else {
                        return JediCylinder.State.Unknown;
                    }


            }

            @Override
            public void setExtendedState(State jediCylinderState) {

                    if (jediCylinderState == JediCylinder.State.Extended) {
                        doubleSolenoid.set(Value.kForward);
                    } else if (jediCylinderState == JediCylinder.State.Retracted) {
                        doubleSolenoid.set(Value.kReverse);
                    } else {
                        doubleSolenoid.set(Value.kOff);
                    }


            }
        };
    }

}
