// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.Parts6;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmDefaultCommand;
import frc.robot.framework.controls.ManualSpeedControl;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ClawState;
import frc.robot.subsystems.Arm.ElevatorTiltState;

/** Add your docs here. */
public class SetManualControl extends ArmDefaultCommand {

    private final ManualSpeedControl manualElevatorSpeedControl;
    private final ManualSpeedControl manualElbowSpeedControl;
    private final ManualSpeedControl manualGrabberSpeedControl;
    private final Trigger clawToggleTrigger;
    private final Trigger eightTrigger;
    private final Trigger sixTrigger;
    private final Trigger twoTrigger;
    private final Trigger noneTrigger;
    private final Trigger dancePartyTrigger;

    private boolean lastClawTrigger = false;
    private boolean lastEightTrigger = false;
    private boolean lastSixTrigger = false;
    private boolean lastTwoTrigger = false;
    private boolean lastNoneTrigger = false;
    private boolean lastDancePartyTrigger = false;

    public SetManualControl(
            Arm arm,
            ManualSpeedControl manualElevatorSpeedControl,
            ManualSpeedControl manualElbowSpeedControl,
            ManualSpeedControl manualGrabberSpeedControl,
            Trigger clawToggleTrigger,
            Trigger eightTrigger,
            Trigger sixTrigger,
            Trigger twoTrigger,
            Trigger noneTrigger,
            Trigger dancePartyTrigger) {
        super(arm);
        this.manualElevatorSpeedControl = manualElevatorSpeedControl;
        this.manualElbowSpeedControl = manualElbowSpeedControl;
        this.manualGrabberSpeedControl = manualGrabberSpeedControl;
        this.clawToggleTrigger = clawToggleTrigger;
        this.eightTrigger = eightTrigger;
        this.sixTrigger = sixTrigger;
        this.twoTrigger = twoTrigger;
        this.noneTrigger = noneTrigger;
        this.dancePartyTrigger = dancePartyTrigger;
    }

    @Override
    public void initialize() {
   }

    @Override
    public void execute() {

        arm.setElevatorVelocity(manualElevatorSpeedControl.getManualSpeed());
        arm.setElbowVelocity(manualElbowSpeedControl.getManualSpeed());
        arm.setAverageVelocityPerSecond(manualGrabberSpeedControl.getManualSpeed());
    }

}
