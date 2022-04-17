package com.kennedyrobotics.hardware.motors.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.kennedyrobotics.hardware.sensors.ctre.ICANCoder;

public class VictorSPX extends com.ctre.phoenix.motorcontrol.can.VictorSPX implements IVictorSPX {
    /**
     * Constructor
     *
     * @param deviceNumber [0,62]
     */
    public VictorSPX(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(ICANCoder canCoderRef, int remoteOrdinal, int timeoutMs) {
        return configRemoteFeedbackFilter(canCoderRef.getDeviceID(), RemoteSensorSource.CANCoder, remoteOrdinal, timeoutMs);
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(ICANCoder canCoderRef, int remoteOrdinal) {
        return configRemoteFeedbackFilter(canCoderRef, remoteOrdinal, 0);
    }
}
