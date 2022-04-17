package com.kennedyrobotics.hardware.motors.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

public class GhostTalonSRX extends GhostBaseMotorController implements ITalonSRX {

    public GhostTalonSRX(int id) {
        super(id);
    }

    @Override
    public void set(TalonSRXControlMode mode, double value) {

    }

    @Override
    public void set(TalonSRXControlMode mode, double demand0, DemandType demand1Type, double demand1) {

    }

    @Override
    public ISensorCollection getISensorCollection() {
        return new ISensorCollection() {
            @Override
            public int getAnalogIn() {
                return 0;
            }

            @Override
            public ErrorCode setAnalogPosition(int newPosition, int timeoutMs) {
                return ErrorCode.OK;
            }

            @Override
            public int getAnalogInRaw() {
                return 0;
            }

            @Override
            public int getAnalogInVel() {
                return 0;
            }

            @Override
            public int getQuadraturePosition() {
                return 0;
            }

            @Override
            public ErrorCode setQuadraturePosition(int newPosition, int timeoutMs) {
                return ErrorCode.OK;
            }

            @Override
            public ErrorCode syncQuadratureWithPulseWidth(int bookend0, int bookend1, boolean bCrossZeroOnInterval, int offset, int timeoutMs) {
                return ErrorCode.OK;
            }

            @Override
            public ErrorCode syncQuadratureWithPulseWidth(int bookend0, int bookend1, boolean bCrossZeroOnInterval) {
                return ErrorCode.OK;
            }

            @Override
            public int getQuadratureVelocity() {
                return 0;
            }

            @Override
            public int getPulseWidthPosition() {
                return 0;
            }

            @Override
            public ErrorCode setPulseWidthPosition(int newPosition, int timeoutMs) {
                return ErrorCode.OK;
            }

            @Override
            public int getPulseWidthVelocity() {
                return 0;
            }

            @Override
            public int getPulseWidthRiseToFallUs() {
                return 0;
            }

            @Override
            public int getPulseWidthRiseToRiseUs() {
                return 0;
            }

            @Override
            public boolean getPinStateQuadA() {
                return false;
            }

            @Override
            public boolean getPinStateQuadB() {
                return false;
            }

            @Override
            public boolean getPinStateQuadIdx() {
                return false;
            }

            @Override
            public boolean isFwdLimitSwitchClosed() {
                return false;
            }

            @Override
            public boolean isRevLimitSwitchClosed() {
                return false;
            }
        };
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(TalonSRXFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitConfigs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs) {
        return 0;
    }

    @Override
    public double getOutputCurrent() {
        return 0;
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(VelocityMeasPeriod period, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitConfigs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configPeakCurrentLimit(int amps, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configPeakCurrentLimit(int amps) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configPeakCurrentDuration(int milliseconds, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configPeakCurrentDuration(int milliseconds) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configContinuousCurrentLimit(int amps, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configContinuousCurrentLimit(int amps) {
        return ErrorCode.OK;
    }

    @Override
    public void enableCurrentLimit(boolean enable) {

    }

    @Override
    public void getPIDConfigs(TalonSRXPIDSetConfiguration pid, int pidIdx, int timeoutMs) {

    }

    @Override
    public void getPIDConfigs(TalonSRXPIDSetConfiguration pid) {

    }

    @Override
    public ErrorCode configAllSettings(TalonSRXConfiguration allConfigs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAllSettings(TalonSRXConfiguration allConfigs) {
        return ErrorCode.OK;
    }

    @Override
    public void getAllConfigs(TalonSRXConfiguration allConfigs, int timeoutMs) {

    }

    @Override
    public void getAllConfigs(TalonSRXConfiguration allConfigs) {

    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs) {
        return ErrorCode.OK;
    }
}
