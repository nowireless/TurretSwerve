package com.kennedyrobotics.hardware.motors.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class GhostTalonFX extends GhostBaseMotorController implements ITalonFX {

    public GhostTalonFX(int id) {
        super(id);
    }

    @Override
    public void set(TalonFXControlMode mode, double value) {

    }

    @Override
    public void set(TalonFXControlMode mode, double demand0, DemandType demand1Type, double demand1) {

    }

    @Override
    public void setInverted(TalonFXInvertType invertType) {

    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(TalonFXFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitCfg, int timeoutMs) {
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
    public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitCfg) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configStatorCurrentLimit(StatorCurrentLimitConfiguration currLimitCfg, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configStatorCurrentLimit(StatorCurrentLimitConfiguration currLimitCfg) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configGetSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitConfigsToFill, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configGetSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitConfigsToFill) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configGetStatorCurrentLimit(StatorCurrentLimitConfiguration currLimitConfigsToFill, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configGetStatorCurrentLimit(StatorCurrentLimitConfiguration currLimitConfigsToFill) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotorCommutation(MotorCommutation motorCommutation, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotorCommutation(MotorCommutation motorCommutation) {
        return ErrorCode.OK;
    }

    @Override
    public MotorCommutation configGetMotorCommutation(int timeoutMs) {
        return MotorCommutation.Trapezoidal;
    }

    @Override
    public MotorCommutation configGetMotorCommutation() {
        return MotorCommutation.Trapezoidal;
    }

    @Override
    public ErrorCode configIntegratedSensorAbsoluteRange(AbsoluteSensorRange absoluteSensorRange, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configIntegratedSensorAbsoluteRange(AbsoluteSensorRange absoluteSensorRange) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configIntegratedSensorOffset(double offsetDegrees, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configIntegratedSensorOffset(double offsetDegrees) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configIntegratedSensorInitializationStrategy(SensorInitializationStrategy initializationStrategy, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configIntegratedSensorInitializationStrategy(SensorInitializationStrategy initializationStrategy) {
        return ErrorCode.OK;
    }

    @Override
    public ITalonFXSensorCollection getISensorCollection() {
        return new ITalonFXSensorCollection() {
            @Override
            public double getIntegratedSensorPosition() {
                return 0;
            }

            @Override
            public double getIntegratedSensorAbsolutePosition() {
                return 0;
            }

            @Override
            public double getIntegratedSensorVelocity() {
                return 0;
            }

            @Override
            public ErrorCode setIntegratedSensorPosition(double newPosition, int timeoutMs) {
                return ErrorCode.OK;
            }

            @Override
            public ErrorCode setIntegratedSensorPositionToAbsolute(int timeoutMs) {
                return ErrorCode.OK;
            }

            @Override
            public int isFwdLimitSwitchClosed() {
                return 0;
            }

            @Override
            public int isRevLimitSwitchClosed() {
                return 0;
            }
        };
    }

    @Override
    public void getPIDConfigs(TalonFXPIDSetConfiguration pid, int pidIdx, int timeoutMs) {

    }

    @Override
    public void getPIDConfigs(TalonFXPIDSetConfiguration pid) {

    }

    @Override
    public ErrorCode configAllSettings(TalonFXConfiguration allConfigs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAllSettings(TalonFXConfiguration allConfigs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getAllConfigs(TalonFXConfiguration allConfigs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getAllConfigs(TalonFXConfiguration allConfigs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs) {
        return ErrorCode.OK;
    }
}
