package com.kennedyrobotics.hardware.leds.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle;

public class GhostCANdle implements ICANdle {
    @Override
    public double getBusVoltage() {
        return 0;
    }

    @Override
    public double get5VRailVoltage() {
        return 0;
    }

    @Override
    public double getCurrent() {
        return 0;
    }

    @Override
    public double getTemperature() {
        return 0;
    }

    @Override
    public double getVBatModulation() {
        return 0;
    }

    @Override
    public int getMaxSimultaneousAnimationCount() {
        return 0;
    }

    @Override
    public ErrorCode animate(Animation animation) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode animate(Animation animation, int animSlot) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode clearAnimation(int animSlot) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setLEDs(int r, int g, int b, int w, int startIdx, int count) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setLEDs(int r, int g, int b) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode modulateVBatOutput(double dutyCyclePrcnt) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configLOSBehavior(boolean disableWhenLOS, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configLOSBehavior(boolean disableWhenLOS) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configLEDType(CANdle.LEDStripType type, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configLEDType(CANdle.LEDStripType type) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configBrightnessScalar(double brightness, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configBrightnessScalar(double brightness) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configStatusLedState(boolean disableWhenRunning, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configStatusLedState(boolean disableWhenRunning) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVBatOutput(CANdle.VBatOutputMode mode, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVBatOutput(CANdle.VBatOutputMode mode) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configV5Enabled(boolean enable5V, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configV5Enabled(boolean enable5V) {
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
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal) {
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
    public ErrorCode configSetCustomParam(int paramIndex, int value, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSetCustomParam(int paramIndex, int value) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configFactoryDefault(int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configFactoryDefault() {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getFaults(CANdleFaults toFill) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getStickyFaults(CANdleStickyFaults toFill) {
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
    public boolean hasResetOccurred() {
        return false;
    }

    @Override
    public ErrorCode setStatusFramePeriod(CANdleStatusFrame frame, int periodMs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(CANdleStatusFrame frame, int periodMs) {
        return ErrorCode.OK;
    }

    @Override
    public int getStatusFramePeriod(CANdleStatusFrame frame, int timeoutMs) {
        return 0;
    }

    @Override
    public int getStatusFramePeriod(CANdleStatusFrame frame) {
        return 0;
    }

    @Override
    public ErrorCode setControlFramePeriod(CANdleControlFrame frame, int periodMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAllSettings(CANdleConfiguration allConfigs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAllSettings(CANdleConfiguration allConfigs) {
        return ErrorCode.OK;
    }

    @Override
    public void getAllConfigs(CANdleConfiguration allConfigs, int timeoutMs) {

    }

    @Override
    public void getAllConfigs(CANdleConfiguration allConfigs) {

    }

    @Override
    public ErrorCode getLastError() {
        return ErrorCode.OK;
    }
}
