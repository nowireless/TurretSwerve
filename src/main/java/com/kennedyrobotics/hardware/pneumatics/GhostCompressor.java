package com.kennedyrobotics.hardware.pneumatics;

import com.kennedyrobotics.hardware.pneumatics.ICompressor;
import edu.wpi.first.wpilibj.CompressorConfigType;

public class GhostCompressor implements ICompressor {

    private CompressorConfigType type = CompressorConfigType.Disabled;

    @Override
    public boolean enabled() {
        return type != CompressorConfigType.Disabled;
    }

    @Override
    public boolean getPressureSwitchValue() {
        return false;
    }

    @Override
    public double getCurrent() {
        return 0;
    }

    @Override
    public double getAnalogVoltage() {
        return 0;
    }

    @Override
    public double getPressure() {
        return 0;
    }

    @Override
    public void disable() {
        type = CompressorConfigType.Disabled;
    }

    @Override
    public void enableDigital() {
        type = CompressorConfigType.Digital;
    }

    @Override
    public void enableAnalog(double minPressure, double maxPressure) {
        type = CompressorConfigType.Analog;
    }

    @Override
    public void enableHybrid(double minPressure, double maxPressure) {
        type = CompressorConfigType.Hybrid;
    }

    @Override
    public CompressorConfigType getConfigType() {
        return type;
    }
}
