package com.revrobotics;

import com.kennedyrobotics.hardware.motors.rev.ISparkMaxLimitSwitch;

public class SparkMaxLimitSwitchShim extends SparkMaxLimitSwitch implements ISparkMaxLimitSwitch {
    private SparkMaxLimitSwitchShim(CANSparkMax device, Direction direction, Type switchType) {
        super(device, direction, switchType);
    }

    @Override
    public Type getType() {
        return m_switchType;
    }

    //
    // Static methods to get around package private enums
    //
    public static ISparkMaxLimitSwitch makeReverseLimitSwitch(CANSparkMax device, Type switchType) {
        return new SparkMaxLimitSwitchShim(device, SparkMaxLimitSwitch.Direction.kReverse, switchType);
    }

    public static ISparkMaxLimitSwitch makeForwardLimitSwitch(CANSparkMax device, Type switchType) {
        return new SparkMaxLimitSwitchShim(device, Direction.kForward, switchType);
    }
}
