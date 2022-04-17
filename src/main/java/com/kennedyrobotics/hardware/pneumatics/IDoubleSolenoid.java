package com.kennedyrobotics.hardware.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface IDoubleSolenoid {

    /**
     * Set the value of a solenoid.
     *
     * @param value The value to set (Off, Forward, Reverse)
     */
    void set(final DoubleSolenoid.Value value);

    /**
     * Read the current value of the solenoid.
     *
     * @return The current value of the solenoid.
     */
    DoubleSolenoid.Value get();

    /**
     * Toggle the value of the solenoid.
     *
     * <p>If the solenoid is set to forward, it'll be set to reverse. If the solenoid is set to
     * reverse, it'll be set to forward. If the solenoid is set to off, nothing happens.
     */
    void toggle();

    /**
     * Get the forward channel.
     *
     * @return the forward channel.
     */
    int getFwdChannel();

    /**
     * Get the reverse channel.
     *
     * @return the reverse channel.
     */
    int getRevChannel();

    /**
     * Check if the forward solenoid is Disabled. If a solenoid is shorted, it is added to the
     * DisabledList and disabled until power cycle, or until faults are cleared.
     *
     * @return If solenoid is disabled due to short.
     */
    boolean isFwdSolenoidDisabled();

    /**
     * Check if the reverse solenoid is Disabled. If a solenoid is shorted, it is added to the
     * DisabledList and disabled until power cycle, or until faults are cleared.
     *
     * @return If solenoid is disabled due to short.
     */
    boolean isRevSolenoidDisabled();
}
