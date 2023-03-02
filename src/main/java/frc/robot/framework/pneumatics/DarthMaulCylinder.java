// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.pneumatics;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DarthMaulCylinder extends SubsystemBase {

  private final JediCylinder leftJediCylinder;
  private final JediCylinder rightJediCylinder;

  public DarthMaulCylinder(
      JediCylinder leftJediCylinder,
      JediCylinder rightJediCylinder) {
    this.leftJediCylinder = leftJediCylinder;
    this.rightJediCylinder = rightJediCylinder;
  }

  public DarthMaulCylinder.State getDarthMaulCylinderState() {
    if (leftJediCylinder.getExtendedState() == JediCylinder.State.Extended &&
        rightJediCylinder.getExtendedState() == JediCylinder.State.Extended) {
      return DarthMaulCylinder.State.DuelOfTheFates;
    } else if (leftJediCylinder.getExtendedState() == JediCylinder.State.Extended &&
        rightJediCylinder.getExtendedState() == JediCylinder.State.Retracted) {
      return DarthMaulCylinder.State.Tatooine;
    } else if (leftJediCylinder.getExtendedState() == JediCylinder.State.Retracted &&
        rightJediCylinder.getExtendedState() == JediCylinder.State.Extended) {
      return DarthMaulCylinder.State.ShortSaber;
    } else {
      return DarthMaulCylinder.State.NoLegs;
    }
  }

  public void setDarthMaulCylinderState(DarthMaulCylinder.State darthMaulCylinderState) {
    if (darthMaulCylinderState == DarthMaulCylinder.State.DuelOfTheFates) {
      leftJediCylinder.setExtendedState(JediCylinder.State.Extended);
      rightJediCylinder.setExtendedState(JediCylinder.State.Extended);
    } else if (darthMaulCylinderState == DarthMaulCylinder.State.Tatooine) {
      leftJediCylinder.setExtendedState(JediCylinder.State.Extended);
      rightJediCylinder.setExtendedState(JediCylinder.State.Retracted);
    } else if (darthMaulCylinderState == DarthMaulCylinder.State.ShortSaber) {
      leftJediCylinder.setExtendedState(JediCylinder.State.Retracted);
      rightJediCylinder.setExtendedState(JediCylinder.State.Extended);
    } else {
      leftJediCylinder.setExtendedState(JediCylinder.State.Retracted);
      rightJediCylinder.setExtendedState(JediCylinder.State.Retracted);
    }
  }

  public enum State {
    Tatooine,
    ShortSaber,
    DuelOfTheFates,
    NoLegs
  }
}
