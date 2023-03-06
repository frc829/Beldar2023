// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.framework.pneumatics.JediCylinder;

public class Claw extends SubsystemBase {
  private final JediCylinder claw;

  public Claw(JediCylinder claw) {
    this.claw = claw;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString(
        "Claw Position",
        getState().name());
  }

  public enum State {
    CUBE, CONE
  }

  public Claw.State getState() {
    JediCylinder.State jediCylinderState = this.claw.getExtendedState();
    return jediCylinderState == JediCylinder.State.Extended ? State.CUBE : State.CONE;
  }

  public CommandBase createSetStateCommand() {
    Runnable setState = new Runnable() {
      @Override
      public void run() {

        JediCylinder.State currentJediCylinderState = claw.getExtendedState();
        JediCylinder.State jediCylinderStateToSet = currentJediCylinderState == JediCylinder.State.Extended
            ? JediCylinder.State.Retracted
            : JediCylinder.State.Extended;
        claw.setExtendedState(jediCylinderStateToSet);
      }
    };
    return Commands.runOnce(setState, this);
  }

  public CommandBase createSetStateCommand(Claw.State clawState) {

    CommandBase setStateCommand = new CommandBase() {
      @Override
          public void execute() {
            JediCylinder.State jediCylinderState = clawState == State.CUBE ? JediCylinder.State.Extended
            : JediCylinder.State.Retracted;
        claw.setExtendedState(jediCylinderState);
          }

      @Override
          public boolean isFinished() {
              return true;
          }
    };

    setStateCommand.addRequirements(this);

    return setStateCommand;

  }

  public CommandBase createOpenCommand(){
    return Commands.runOnce(
      () -> claw.setExtendedState(JediCylinder.State.Extended), 
      this);
  }
}
