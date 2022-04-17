package com.revrobotics;

import com.kennedyrobotics.hardware.motors.rev.ISparkMaxAnalogSensor;

public class SparkMaxAnalogSensorShim extends SparkMaxAnalogSensor implements ISparkMaxAnalogSensor {
    public SparkMaxAnalogSensorShim(CANSparkMax sparkMax, Mode mode) {
        super(sparkMax, mode);
    }

    @Override
    public Mode getMode() {
        return mode;
    }
}
