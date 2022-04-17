package com.kennedyrobotics.hardware.pneumatics;

public interface ISolenoid {

    /**
     * Set the value of a solenoid.
     *
     * @param on True will turn the solenoid output on. False will turn the solenoid output off.
     */
    void set(boolean on);

    /**
     * Read the current value of the solenoid.
     *
     * @return True if the solenoid output is on or false if the solenoid output is off.
     */
    boolean get();

    /**
     * Toggle the value of the solenoid.
     *
     * <p>If the solenoid is set to on, it'll be turned off. If the solenoid is set to off, it'll be
     * turned on.
     */
    void toggle();

    /**
     * Get the channel this solenoid is connected to.
     *
     * @return The channel this solenoid is connected to.
     */
    int getChannel();

    /**
     * Check if solenoid is DisabledListed. If a solenoid is shorted, it is added to the Disabled List
     * and disabled until power cycle, or until faults are cleared.
     *
     * @return If solenoid is disabled due to short.
     */
    boolean isDisabled();

    /**
     * Set the pulse duration in the PCM. This is used in conjunction with the startPulse method to
     * allow the PCM to control the timing of a pulse. The timing can be controlled in 0.01 second
     * increments.
     *
     * @param durationSeconds The duration of the pulse, from 0.01 to 2.55 seconds.
     * @see #startPulse()
     */
    void setPulseDuration(double durationSeconds);

    /**
     * Trigger the PCM to generate a pulse of the duration set in setPulseDuration.
     *
     * @see #setPulseDuration(double)
     */
    void startPulse();
}
