package com.kennedyrobotics.hardware.motors.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.kennedyrobotics.hardware.sensors.ctre.ICANCoder;

public class TalonFX extends com.ctre.phoenix.motorcontrol.can.TalonFX implements ITalonFX {

    // This is a hack, but is easy.
    private final ITalonFXSensorCollection m_talonFxSensorCollection;

    public TalonFX(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
        m_talonFxSensorCollection = new TalonFXSensorCollector(this);
    }

    public TalonFX(int deviceNumber) {
        super(deviceNumber);
        m_talonFxSensorCollection = new TalonFXSensorCollector(this);
    }

    @Override
    public TalonFXSensorCollection getSensorCollection() {
        return super.getSensorCollection();
    }

    @Override
    public ITalonFXSensorCollection getISensorCollection() {
        return m_talonFxSensorCollection;
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
