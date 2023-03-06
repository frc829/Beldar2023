// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.framework.pneumatics.DarthMaulCylinder;

public class ElevatorTilt extends SubsystemBase {

  private final DarthMaulCylinder elevatorTilt;
  private final MechanismLigament2d fakeElevatorMech2d;
  private final MechanismLigament2d elevatorMech2d;
  private double tiltAngleDegreesSim = 0;

  public ElevatorTilt(DarthMaulCylinder elevatorTilt, MechanismLigament2d fakeElevatorMech2d,
      MechanismLigament2d elevatorMech2d) {
    this.elevatorTilt = elevatorTilt;
    this.fakeElevatorMech2d = fakeElevatorMech2d;
    this.elevatorMech2d = elevatorMech2d;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString(
        "Elevator Tilt State",
        this.elevatorTilt.getDarthMaulCylinderState().name());

    adjustTile(getState());

    this.elevatorMech2d.setAngle(90 + tiltAngleDegreesSim);
    this.fakeElevatorMech2d.setAngle(90 + tiltAngleDegreesSim);
  }

  public enum State {
    NONE, TWO, SIX, EIGHT
  }

  private void adjustTile(State elevatorTiltState) {

    if (elevatorTiltState == State.TWO) {
      if (tiltAngleDegreesSim > 11) {
        tiltAngleDegreesSim -= 0.020 * 90;
      } else if (tiltAngleDegreesSim < 9) {
        tiltAngleDegreesSim += 0.020 * 90;
      }
    } else if (elevatorTiltState == State.SIX) {
      if (tiltAngleDegreesSim > 46) {
        tiltAngleDegreesSim -= 0.020 * 90;
      } else if (tiltAngleDegreesSim < 44) {
        tiltAngleDegreesSim += 0.020 * 90;
      }
    } else if (elevatorTiltState == State.EIGHT) {
      if (tiltAngleDegreesSim > 76) {
        tiltAngleDegreesSim -= 0.020 * 90;
      } else if (tiltAngleDegreesSim < 74) {
        tiltAngleDegreesSim += 0.020 * 90;
      }
    } else if (elevatorTiltState == State.NONE) {
      if (tiltAngleDegreesSim > 1) {
        tiltAngleDegreesSim -= 0.020 * 90;
      } else if (tiltAngleDegreesSim < -1) {
        tiltAngleDegreesSim += 0.020 * 90;
      }
    }
  }

  public State getState() {
    DarthMaulCylinder.State darthMaulCylinderState = elevatorTilt.getDarthMaulCylinderState();
    if (darthMaulCylinderState == DarthMaulCylinder.State.NoLegs) {
      return State.NONE;
    } else if (darthMaulCylinderState == DarthMaulCylinder.State.ShortSaber) {
      return State.TWO;
    } else if (darthMaulCylinderState == DarthMaulCylinder.State.Tatooine) {
      return State.SIX;
    } else {
      return State.EIGHT;
    }
  }

  public CommandBase createSetStateCommand(State elevatorTiltState) {


    CommandBase setCommand = new CommandBase() {
      @Override
      public void execute() {
        if (elevatorTiltState == State.NONE) {
          elevatorTilt.setDarthMaulCylinderState(DarthMaulCylinder.State.NoLegs);
        } else if (elevatorTiltState == State.TWO) {
          elevatorTilt.setDarthMaulCylinderState(DarthMaulCylinder.State.ShortSaber);
        } else if (elevatorTiltState == State.SIX) {
          elevatorTilt.setDarthMaulCylinderState(DarthMaulCylinder.State.Tatooine);
        } else {
          elevatorTilt.setDarthMaulCylinderState(DarthMaulCylinder.State.DuelOfTheFates);
        }
      }
      @Override
          public boolean isFinished() {
              return true;
          }
    };

    setCommand.addRequirements(this);

    return setCommand;

  }
}
