package com.kennedyrobotics.hardware.pneumatics;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Solenoid extends edu.wpi.first.wpilibj.Solenoid implements ISolenoid {

    /**
     * Constructs a solenoid for a default module and specified type.
     *
     * @param moduleType The module type to use.
     * @param channel The channel the solenoid is on.
     */
    public Solenoid(final PneumaticsModuleType moduleType, final int channel) {
        super(moduleType, channel);
    }

    /**
     * Constructs a solenoid for a specified module and type.
     *
     * @param module The module ID to use.
     * @param moduleType The module type to use.
     * @param channel The channel the solenoid is on.
     */
    public Solenoid(final int module, final PneumaticsModuleType moduleType, final int channel) {
        super(module, moduleType, channel);
    }
}
