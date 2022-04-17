package com.kennedyrobotics.hardware.sensors.ctre;

public class CANCoder extends com.ctre.phoenix.sensors.CANCoder implements ICANCoder {
    public CANCoder(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
    }

    public CANCoder(int deviceNumber) {
        super(deviceNumber);
    }
}
