package frc.robot.framework.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public interface HalfControllerAxis {

    public double getAxisValue();

    public static HalfControllerAxis getAxisControl(
            CommandXboxController xboxController,
            PositiveOnlyAxisType positiveOnlyAxisType,
            boolean isInverted,
            double xboxControllerAxisDeadband) {
        return new HalfControllerAxis() {

            @Override
            public double getAxisValue() {
                double rawAxisValue = positiveOnlyAxisType == PositiveOnlyAxisType.LeftTriggerAxis
                        ? xboxController.getRawAxis(XboxController.Axis.kLeftTrigger.value)
                        : xboxController.getRawAxis(XboxController.Axis.kRightTrigger.value);
                double adjustedAxisValue = MathUtil.applyDeadband(rawAxisValue, xboxControllerAxisDeadband);
                adjustedAxisValue = adjustedAxisValue > 1.0 ? 1.0 : adjustedAxisValue;
                adjustedAxisValue = isInverted ? -adjustedAxisValue : adjustedAxisValue;
                return adjustedAxisValue;
            }
        };
    }

    public enum PositiveOnlyAxisType {
        LeftTriggerAxis,
        RightTriggerAxis
    }

}
