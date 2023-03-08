// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.framework.pneumatics.JediCylinder;

public class Claw extends SubsystemBase {
  private final JediCylinder claw;

  public enum State {
    CUBE, CONE
  }

  public Claw(JediCylinder claw) {
    this.claw = claw;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString(
        "Claw Position",
        getState().name());
  }

  // Suppliers
  public Claw.State getState() {
    JediCylinder.State jediCylinderState = this.claw.getExtendedState();
    return jediCylinderState == JediCylinder.State.Extended ? State.CUBE : State.CONE;
  }

  public boolean hasCone() {
    return getState() == State.CONE;
  }

  // Consumers

  public void setState(Claw.State clawState) {
    JediCylinder.State jediCylinderState = clawState == Claw.State.CONE ? JediCylinder.State.Retracted
        : JediCylinder.State.Extended;
    claw.setExtendedState(jediCylinderState);
  }

  // Commands

  public CommandBase createIdleCommand() {
    CommandBase idleCommand = new CommandBase() {

      @Override
      public void initialize() {
        String lastCommand = SmartDashboard.getString("Claw Command Current", "Idle");
        SmartDashboard.putString("Claw Command Last", lastCommand);
        SmartDashboard.putString("Claw Command Current", "Idle");
      }

    };

    idleCommand.addRequirements(this);
    return idleCommand;
  }

  public CommandBase createSetStateCommand(Claw.State clawState) {

    CommandBase setStateCommand = new CommandBase() {

      @Override
      public void initialize() {
        String lastCommand = SmartDashboard.getString("Claw Command Current", "Idle");
        SmartDashboard.putString("Claw Command Last", lastCommand);
        SmartDashboard.putString("Claw Command Current", "SetState: " + clawState.name());
      }

      @Override
      public void execute() {
        setState(clawState);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };

    setStateCommand.addRequirements(this);
    return setStateCommand;

  }
}
