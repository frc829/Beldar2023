// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.simulation;

/** Add your docs here. */
/**
 * Holds information about a simulated device.
 */
public abstract class SimProfile {
    private long _lastTime;
    private boolean _running = false;

    /**
     * Runs the simulation profile.
     * Implemented by device-specific profiles.
     */
    abstract public void run();

    /**
     * Returns the time since last call, in milliseconds.
     */
    protected double getPeriod() {
        // set the start time if not yet running
        if (!_running) {
            _lastTime = System.nanoTime();
            _running = true;
        }

        long now = System.nanoTime();
        final double period = (now - _lastTime) / 1e9;
        _lastTime = now;

        return period;
    }
}
