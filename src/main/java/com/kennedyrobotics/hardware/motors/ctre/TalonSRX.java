package com.kennedyrobotics.hardware.motors.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.kennedyrobotics.hardware.sensors.ctre.ICANCoder;

public class TalonSRX extends com.ctre.phoenix.motorcontrol.can.TalonSRX implements ITalonSRX {

    // This is a hack, but is easy.
    private final SensorCollection m_sensorCollection;

    /**
     * Constructor for TalonSRX object
     *
     * @param deviceNumber CAN Device ID of Device
     */
    public TalonSRX(int deviceNumber) {
        super(deviceNumber);
        m_sensorCollection = new SensorCollection(this);
    }

    @Override
    public ISensorCollection getISensorCollection() {
        return m_sensorCollection;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(ICANCoder canCoderRef, int remoteOrdinal, int timeoutMs) {
        return configRemoteFeedbackFilter(canCoderRef.getDeviceID(), RemoteSensorSource.CANCoder, remoteOrdinal, timeoutMs);
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(ICANCoder canCoderRef, int remoteOrdinal) {
        return configRemoteFeedbackFilter(canCoderRef, remoteOrdinal);
    }
}
