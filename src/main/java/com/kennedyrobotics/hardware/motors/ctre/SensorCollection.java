package com.kennedyrobotics.hardware.motors.ctre;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public class SensorCollection extends com.ctre.phoenix.motorcontrol.SensorCollection implements ISensorCollection {

    /**
     * Constructor for SensorCollection
     *
     * @param motorController Motor Controller to connect Collection to
     */
    public SensorCollection(BaseTalon motorController) {
        super(motorController);
    }
}
