package com.kennedyrobotics.hardware.sensors.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.sensors.*;

public class GhostCANCoder implements ICANCoder {

    private final int m_id;

    public GhostCANCoder(int id) {
        m_id = id;
    }

    @Override
    public int getDeviceID() {
        return m_id;
    }

    @Override
    public double getPosition() {
        return 0;
    }

    @Override
    public ErrorCode setPosition(double newPosition, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setPosition(double newPosition) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setPositionToAbsolute(int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setPositionToAbsolute() {
        return ErrorCode.OK;
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public double getAbsolutePosition() {
        return 0;
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(SensorVelocityMeasPeriod period, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(SensorVelocityMeasPeriod period) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVelocityMeasurementWindow(int windowSize) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAbsoluteSensorRange(AbsoluteSensorRange absoluteSensorRange, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAbsoluteSensorRange(AbsoluteSensorRange absoluteSensorRange) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMagnetOffset(double offsetDegrees, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMagnetOffset(double offsetDegrees) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSensorInitializationStrategy(SensorInitializationStrategy initializationStrategy, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSensorInitializationStrategy(SensorInitializationStrategy initializationStrategy) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configFeedbackCoefficient(double sensorCoefficient, String unitString, SensorTimeBase sensorTimeBase, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configFeedbackCoefficient(double sensorCoefficient, String unitString, SensorTimeBase sensorTimeBase) {
        return ErrorCode.OK;
    }

    @Override
    public double getBusVoltage() {
        return 0;
    }

    @Override
    public MagnetFieldStrength getMagnetFieldStrength() {
        return MagnetFieldStrength.Good_GreenLED;
    }

    @Override
    public ErrorCode configSensorDirection(boolean bSensorDirection, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSensorDirection(boolean bSensorDirection) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getLastError() {
        return ErrorCode.OK;
    }

    @Override
    public String getLastUnitString() {
        return "Degrees"; // Is this right?
    }

    @Override
    public double getLastTimestamp() {
        return 0;
    }

    @Override
    public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSetCustomParam(int newValue, int paramIndex) {
        return ErrorCode.OK;
    }

    @Override
    public int configGetCustomParam(int paramIndex, int timeoutMs) {
        return 0;
    }

    @Override
    public int configGetCustomParam(int paramIndex) {
        return 0;
    }

    @Override
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal) {
        return ErrorCode.OK;
    }

    @Override
    public double configGetParameter(ParamEnum param, int ordinal, int timeoutMs) {
        return 0;
    }

    @Override
    public double configGetParameter(ParamEnum param, int ordinal) {
        return 0;
    }

    @Override
    public ErrorCode setStatusFramePeriod(CANCoderStatusFrame statusFrame, int periodMs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(CANCoderStatusFrame statusFrame, int periodMs) {
        return ErrorCode.OK;
    }

    @Override
    public int getStatusFramePeriod(CANCoderStatusFrame frame, int timeoutMs) {
        return 0;
    }

    @Override
    public int getStatusFramePeriod(CANCoderStatusFrame frame) {
        return 0;
    }

    @Override
    public int getFirmwareVersion() {
        return 0;
    }

    @Override
    public boolean hasResetOccurred() {
        return false;
    }

    @Override
    public ErrorCode getFaults(CANCoderFaults toFill) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getStickyFaults(CANCoderStickyFaults toFill) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode clearStickyFaults(int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode clearStickyFaults() {
        return ErrorCode.OK;
    }

    @Override
    public SensorVelocityMeasPeriod configGetVelocityMeasurementPeriod(int timeoutMs) {
        return SensorVelocityMeasPeriod.Period_1Ms; // IDK if this is the default;
    }

    @Override
    public SensorVelocityMeasPeriod configGetVelocityMeasurementPeriod() {
        return SensorVelocityMeasPeriod.Period_1Ms; // IDK if this is the default;
    }

    @Override
    public int configGetVelocityMeasurementWindow(int timeoutMs) {
        return 0;
    }

    @Override
    public int configGetVelocityMeasurementWindow() {
        return 0;
    }

    @Override
    public AbsoluteSensorRange configGetAbsoluteSensorRange(int timeoutMs) {
        return AbsoluteSensorRange.Signed_PlusMinus180;
    }

    @Override
    public AbsoluteSensorRange configGetAbsoluteSensorRange() {
        return AbsoluteSensorRange.Signed_PlusMinus180;
    }

    @Override
    public double configGetMagnetOffset(int timeoutMs) {
        return 0;
    }

    @Override
    public double configGetMagnetOffset() {
        return 0;
    }

    @Override
    public boolean configGetSensorDirection(int timeoutMs) {
        return false;
    }

    @Override
    public boolean configGetSensorDirection() {
        return false;
    }

    @Override
    public SensorInitializationStrategy configGetSensorInitializationStrategy(int timeoutMs) {
        return SensorInitializationStrategy.BootToAbsolutePosition;
    }

    @Override
    public SensorInitializationStrategy configGetSensorInitializationStrategy() {
        return SensorInitializationStrategy.BootToAbsolutePosition;
    }

    @Override
    public double configGetFeedbackCoefficient(int timeoutMs) {
        return 0;
    }

    @Override
    public double configGetFeedbackCoefficient() {
        return 0;
    }

    @Override
    public String configGetFeedbackUnitString(int timeoutMs) {
        return "Degrees";
    }

    @Override
    public String configGetFeedbackUnitString() {
        return "Degrees";
    }

    @Override
    public SensorTimeBase configGetFeedbackTimeBase(int timeoutMs) {
        return SensorTimeBase.PerMinute;
    }

    @Override
    public SensorTimeBase configGetFeedbackTimeBase() {
        return SensorTimeBase.PerMinute;
    }

    @Override
    public ErrorCode configAllSettings(CANCoderConfiguration allConfigs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAllSettings(CANCoderConfiguration allConfigs) {
        return ErrorCode.OK;
    }

    @Override
    public void getAllConfigs(CANCoderConfiguration allConfigs, int timeoutMs) {

    }

    @Override
    public void getAllConfigs(CANCoderConfiguration allConfigs) {

    }

    @Override
    public ErrorCode configFactoryDefault(int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configFactoryDefault() {
        return ErrorCode.OK;
    }
}
