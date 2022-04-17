package com.kennedyrobotics.hardware.motors.ctre;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public class TalonFXSensorCollector extends TalonFXSensorCollection implements ITalonFXSensorCollection {
    /**
     * Constructor for SensorCollection
     *
     * @param motorController Motor Controller to connect Collection to
     */
    public TalonFXSensorCollector(BaseTalon motorController) {
        super(motorController);
    }
}
