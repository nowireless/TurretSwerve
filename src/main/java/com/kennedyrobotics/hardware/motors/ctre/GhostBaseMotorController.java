package com.kennedyrobotics.hardware.motors.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.kennedyrobotics.hardware.sensors.ctre.ICANCoder;

public class GhostBaseMotorController implements IBaseMotorController {

    private final int m_id;

    public GhostBaseMotorController(int id) {
        m_id = id;
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
    public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull) {
        return ErrorCode.OK;
    }

    @Override
    public void set(ControlMode Mode, double demand) {

    }

    @Override
    public void set(ControlMode Mode, double demand0, DemandType demand1Type, double demand1) {

    }

    @Override
    public void neutralOutput() {

    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {

    }

    @Override
    public void setSensorPhase(boolean PhaseSensor) {

    }

    @Override
    public void setInverted(boolean invert) {

    }

    @Override
    public void setInverted(InvertType invertType) {

    }

    @Override
    public boolean getInverted() {
        return false;
    }

    @Override
    public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public void enableVoltageCompensation(boolean enable) {

    }

    @Override
    public double getBusVoltage() {
        return 0;
    }

    @Override
    public double getMotorOutputPercent() {
        return 0;
    }

    @Override
    public double getMotorOutputVoltage() {
        return 0;
    }

    @Override
    public double getTemperature() {
        return 0;
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSelectedFeedbackCoefficient(double coefficient, int pidIdx, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(ICANCoder canCoderRef, int remoteOrdinal, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(ICANCoder canCoderRef, int remoteOrdinal) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(BaseTalon talonRef, int remoteOrdinal, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(BaseTalon talonRef, int remoteOrdinal) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public double getSelectedSensorPosition(int pidIdx) {
        return 0;
    }

    @Override
    public double getSelectedSensorVelocity(int pidIdx) {
        return 0;
    }

    @Override
    public ErrorCode setSelectedSensorPosition(double sensorPos, int pidIdx, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public int getStatusFramePeriod(StatusFrame frame, int timeoutMs) {
        return 0;
    }

    @Override
    public ErrorCode configPeakOutputReverse(double percentOut) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configNominalOutputForward(double percentOut) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configNominalOutputReverse(double percentOut) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configNeutralDeadband(double percentDeadband) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples) {
        return ErrorCode.OK;
    }

    @Override
    public boolean isVoltageCompensationEnabled() {
        return false;
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSelectedFeedbackCoefficient(double coefficient) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSensorTerm(SensorTerm sensorTerm, RemoteFeedbackDevice feedbackDevice, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSensorTerm(SensorTerm sensorTerm, RemoteFeedbackDevice feedbackDevice) {
        return ErrorCode.OK;
    }

    @Override
    public double getSelectedSensorPosition() {
        return 0;
    }

    @Override
    public double getSelectedSensorVelocity() {
        return 0;
    }

    @Override
    public ErrorCode setSelectedSensorPosition(double sensorPos) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setControlFramePeriod(int frame, int periodMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(int frameValue, int periodMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs) {
        return ErrorCode.OK;
    }

    @Override
    public int getStatusFramePeriod(int frame) {
        return 0;
    }

    @Override
    public int getStatusFramePeriod(StatusFrame frame) {
        return 0;
    }

    @Override
    public int getStatusFramePeriod(StatusFrameEnhanced frame) {
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
    public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int deviceID, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int deviceID) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int deviceID, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public void overrideLimitSwitchesEnable(boolean enable) {

    }

    @Override
    public ErrorCode configForwardSoftLimitThreshold(double forwardSensorLimit, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(double reverseSensorLimit, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public void overrideSoftLimitsEnable(boolean enable) {

    }

    @Override
    public ErrorCode config_kP(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kI(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kD(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kF(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_IntegralZone(int slotIdx, double izone, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAllowableClosedloopError(int slotIdx, double allowableCloseLoopError, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAuxPIDPolarity(boolean invert, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public double getClosedLoopError(int pidIdx) {
        return 0;
    }

    @Override
    public double getIntegralAccumulator(int pidIdx) {
        return 0;
    }

    @Override
    public double getErrorDerivative(int pidIdx) {
        return 0;
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {

    }

    @Override
    public double getClosedLoopTarget(int pidIdx) {
        return 0;
    }

    @Override
    public double getActiveTrajectoryPosition() {
        return 0;
    }

    @Override
    public double getActiveTrajectoryVelocity() {
        return 0;
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(double sensorUnitsPer100ms, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionAcceleration(double sensorUnitsPer100msPerSec, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionSCurveStrength(int curveStrength, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryPeriod(int baseTrajDurationMs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode clearMotionProfileTrajectories() {
        return ErrorCode.OK;
    }

    @Override
    public int getMotionProfileTopLevelBufferCount() {
        return 0;
    }

    @Override
    public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
        return ErrorCode.OK;
    }

    @Override
    public boolean isMotionProfileTopLevelBufferFull() {
        return false;
    }

    @Override
    public void processMotionProfileBuffer() {

    }

    @Override
    public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode changeMotionControlFramePeriod(int periodMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getLastError() {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getFaults(Faults toFill) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getStickyFaults(StickyFaults toFill) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode clearStickyFaults(int timeoutMs) {
        return ErrorCode.OK;
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
    public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public int configGetCustomParam(int paramIndex, int timeoutMs) {
        return 0;
    }

    @Override
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public double configGetParameter(ParamEnum paramEnum, int ordinal, int timeoutMs) {
        return 0;
    }

    @Override
    public double configGetParameter(int paramEnum, int ordinal, int timeoutMs) {
        return 0;
    }

    @Override
    public int getBaseID() {
        return m_id;
    }

    @Override
    public int getDeviceID() {
        return m_id;
    }

    @Override
    public ControlMode getControlMode() {
        return ControlMode.PercentOutput;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int deviceID) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configForwardSoftLimitThreshold(double forwardSensorLimit) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(double reverseSensorLimit) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kP(int slotIdx, double value) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kI(int slotIdx, double value) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kD(int slotIdx, double value) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kF(int slotIdx, double value) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_IntegralZone(int slotIdx, double izone) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAllowableClosedloopError(int slotIdx, double allowableClosedLoopError) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClosedLoopPeakOutput(int slotIdx, double percentOut) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAuxPIDPolarity(boolean invert) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setIntegralAccumulator(double iaccum) {
        return ErrorCode.OK;
    }

    @Override
    public double getClosedLoopError() {
        return 0;
    }

    @Override
    public double getIntegralAccumulator() {
        return 0;
    }

    @Override
    public double getErrorDerivative() {
        return 0;
    }

    @Override
    public double getClosedLoopTarget() {
        return 0;
    }

    @Override
    public double getActiveTrajectoryPosition(int pidIdx) {
        return 0;
    }

    @Override
    public double getActiveTrajectoryVelocity(int pidIdx) {
        return 0;
    }

    @Override
    public double getActiveTrajectoryArbFeedFwd() {
        return 0;
    }

    @Override
    public double getActiveTrajectoryArbFeedFwd(int pidIdx) {
        return 0;
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(double sensorUnitsPer100ms) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionAcceleration(double sensorUnitsPer100msPerSec) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionSCurveStrength(int curveStrength) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode startMotionProfile(BufferedTrajectoryPointStream stream, int minBufferedPts, ControlMode motionProfControlMode) {
        return ErrorCode.OK;
    }

    @Override
    public boolean isMotionProfileFinished() {
        return false;
    }

    @Override
    public ErrorCode clearMotionProfileHasUnderrun() {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryPeriod(int baseTrajDurationMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryInterpolationEnable(boolean enable, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryInterpolationEnable(boolean enable) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configFeedbackNotContinuous(boolean feedbackNotContinuous, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configRemoteSensorClosedLoopDisableNeutralOnLOS(boolean remoteSensorClosedLoopDisableNeutralOnLOS, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClearPositionOnLimitF(boolean clearPositionOnLimitF, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClearPositionOnLimitR(boolean clearPositionOnLimitR, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClearPositionOnQuadIdx(boolean clearPositionOnQuadIdx, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configLimitSwitchDisableNeutralOnLOS(boolean limitSwitchDisableNeutralOnLOS, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSoftLimitDisableNeutralOnLOS(boolean softLimitDisableNeutralOnLOS, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configPulseWidthPeriod_EdgesPerRot(int pulseWidthPeriod_EdgesPerRot, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configPulseWidthPeriod_FilterWindowSz(int pulseWidthPeriod_FilterWindowSz, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode clearStickyFaults() {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSetCustomParam(int newValue, int paramIndex) {
        return ErrorCode.OK;
    }

    @Override
    public int configGetCustomParam(int paramIndex) {
        return 0;
    }

    @Override
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal) {
        return ErrorCode.OK;
    }

    @Override
    public double configGetParameter(ParamEnum param, int ordinal) {
        return 0;
    }

    @Override
    public double configGetParameter(int param, int ordinal) {
        return 0;
    }

    @Override
    public void follow(IMotorController masterToFollow, FollowerType followerType) {

    }

    @Override
    public ErrorCode configureSlot(SlotConfiguration slot) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configureSlot(SlotConfiguration slot, int slotIdx, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public void getSlotConfigs(SlotConfiguration slot, int slotIdx, int timeoutMs) {

    }

    @Override
    public void getSlotConfigs(SlotConfiguration slot) {

    }

    @Override
    public void getFilterConfigs(FilterConfiguration filter, int ordinal, int timeoutMs) {

    }

    @Override
    public void getFilterConfigs(FilterConfiguration filter) {

    }

    @Override
    public void follow(IMotorController masterToFollow) {

    }

    @Override
    public void valueUpdated() {

    }
}
