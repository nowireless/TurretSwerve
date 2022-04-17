package com.kennedyrobotics.hardware.pneumatics;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Compressor extends edu.wpi.first.wpilibj.Compressor implements ICompressor {

    /**
     * Constructs a compressor for a specified module and type.
     *
     * @param module The module ID to use.
     * @param moduleType The module type to use.
     */
    public Compressor(int module, PneumaticsModuleType moduleType) {
        super(module, moduleType);
    }

    /**
     * Constructs a compressor for a default module and specified type.
     *
     * @param moduleType The module type to use.
     */
    public Compressor(PneumaticsModuleType moduleType) {
        super(moduleType);
    }
}
