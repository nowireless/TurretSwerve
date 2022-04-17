package com.kennedyrobotics.hardware.pneumatics;

import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class DoubleSolenoid extends edu.wpi.first.wpilibj.DoubleSolenoid implements IDoubleSolenoid {
    /**
     * Constructs a double solenoid for a default module of a specific module type.
     *
     * @param moduleType The module type to use.
     * @param forwardChannel The forward channel on the module to control.
     * @param reverseChannel The reverse channel on the module to control.
     */
    public DoubleSolenoid(final PneumaticsModuleType moduleType, final int forwardChannel, final int reverseChannel) {
        super(moduleType, forwardChannel, reverseChannel);
    }

    /**
     * Constructs a double solenoid for a specified module of a specific module type.
     *
     * @param module The module of the solenoid module to use.
     * @param moduleType The module type to use.
     * @param forwardChannel The forward channel on the module to control.
     * @param reverseChannel The reverse channel on the module to control.
     */
    @SuppressWarnings("PMD.UseTryWithResources")
    public DoubleSolenoid(final int module,
            final PneumaticsModuleType moduleType,
            final int forwardChannel,
            final int reverseChannel) {
        super(module, moduleType, forwardChannel, reverseChannel);
    }
}
