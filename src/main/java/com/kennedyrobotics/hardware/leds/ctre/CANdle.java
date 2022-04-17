package com.kennedyrobotics.hardware.leds.ctre;

public class CANdle extends com.ctre.phoenix.led.CANdle implements ICANdle {
    public CANdle(int deviceId, String canbus) {
        super(deviceId, canbus);
    }

    public CANdle(int deviceId) {
        super(deviceId);
    }
}
