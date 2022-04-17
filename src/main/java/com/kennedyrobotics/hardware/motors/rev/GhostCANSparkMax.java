package com.kennedyrobotics.hardware.motors.rev;

import com.revrobotics.*;

public class GhostCANSparkMax implements ICANSParkMax {
    private class GhostRelativeEncoder implements RelativeEncoder {
        @Override
        public double getPosition() {
            return 0;
        }

        @Override
        public double getVelocity() {
            return 0;
        }

        @Override
        public REVLibError setPosition(double position) {
            return REVLibError.kOk;
        }

        @Override
        public REVLibError setPositionConversionFactor(double factor) {
            return REVLibError.kOk;
        }

        @Override
        public REVLibError setVelocityConversionFactor(double factor) {
            return REVLibError.kOk;
        }

        @Override
        public double getPositionConversionFactor() {
            return 0;
        }

        @Override
        public double getVelocityConversionFactor() {
            return 0;
        }

        @Override
        public REVLibError setAverageDepth(int depth) {
            return REVLibError.kOk;
        }

        @Override
        public int getAverageDepth() {
            return 0;
        }

        @Override
        public REVLibError setMeasurementPeriod(int period_ms) {
            return REVLibError.kOk;
        }

        @Override
        public int getMeasurementPeriod() {
            return 0;
        }

        @Override
        public int getCountsPerRevolution() {
            return 0;
        }

        @Override
        public REVLibError setInverted(boolean inverted) {
            return REVLibError.kOk;
        }

        @Override
        public boolean getInverted() {
            return false;
        }
    }

    @Override
    public RelativeEncoder getEncoder() {
        return new GhostRelativeEncoder();
    }

    @Override
    public RelativeEncoder getEncoder(SparkMaxRelativeEncoder.Type encoderType, int countsPerRev) {
        return new GhostRelativeEncoder();
    }

    @Override
    public RelativeEncoder getAlternateEncoder(int countsPerRev) {
        return new GhostRelativeEncoder();
    }

    @Override
    public RelativeEncoder getAlternateEncoder(SparkMaxAlternateEncoder.Type encoderType, int countsPerRev) {
        return new GhostRelativeEncoder();
    }

    @Override
    public ISparkMaxAnalogSensor getIAnalog(SparkMaxAnalogSensor.Mode mode) {
        return new ISparkMaxAnalogSensor() {
            @Override
            public SparkMaxAnalogSensor.Mode getMode() {
                return SparkMaxAnalogSensor.Mode.kAbsolute;
            }

            @Override
            public double getVelocity() {
                return 0;
            }

            @Override
            public REVLibError setVelocityConversionFactor(double factor) {
                return REVLibError.kOk;
            }

            @Override
            public double getVelocityConversionFactor() {
                return 0;
            }

            @Override
            public double getVoltage() {
                return 0;
            }

            @Override
            public double getPosition() {
                return 0;
            }

            @Override
            public REVLibError setPositionConversionFactor(double factor) {
                return REVLibError.kOk;
            }

            @Override
            public double getPositionConversionFactor() {
                return 0;
            }

            @Override
            public REVLibError setInverted(boolean inverted) {
                return REVLibError.kOk;
            }

            @Override
            public boolean getInverted() {
                return false;
            }
        };
    }

    @Override
    public ISparkMaxPIDController getISparkPIDController() {
        return new ISparkMaxPIDController() {
            @Override
            public REVLibError setReference(double value, com.revrobotics.CANSparkMax.ControlType ctrl) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setReference(double value, com.revrobotics.CANSparkMax.ControlType ctrl, int pidSlot) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setReference(double value, com.revrobotics.CANSparkMax.ControlType ctrl, int pidSlot, double arbFeedforward) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setReference(double value, com.revrobotics.CANSparkMax.ControlType ctrl, int pidSlot, double arbFeedforward, SparkMaxPIDController.ArbFFUnits arbFFUnits) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setP(double gain) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setP(double gain, int slotID) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setI(double gain) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setI(double gain, int slotID) {
                return null;
            }

            @Override
            public REVLibError setD(double gain) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setD(double gain, int slotID) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setDFilter(double gain) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setDFilter(double gain, int slotID) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setFF(double gain) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setFF(double gain, int slotID) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setIZone(double IZone) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setIZone(double IZone, int slotID) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setOutputRange(double min, double max) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setOutputRange(double min, double max, int slotID) {
                return REVLibError.kOk;
            }

            @Override
            public double getP() {
                return 0;
            }

            @Override
            public double getP(int slotID) {
                return 0;
            }

            @Override
            public double getI() {
                return 0;
            }

            @Override
            public double getI(int slotID) {
                return 0;
            }

            @Override
            public double getD() {
                return 0;
            }

            @Override
            public double getD(int slotID) {
                return 0;
            }

            @Override
            public double getDFilter(int slotID) {
                return 0;
            }

            @Override
            public double getFF() {
                return 0;
            }

            @Override
            public double getFF(int slotID) {
                return 0;
            }

            @Override
            public double getIZone() {
                return 0;
            }

            @Override
            public double getIZone(int slotID) {
                return 0;
            }

            @Override
            public double getOutputMin() {
                return 0;
            }

            @Override
            public double getOutputMin(int slotID) {
                return 0;
            }

            @Override
            public double getOutputMax() {
                return 0;
            }

            @Override
            public double getOutputMax(int slotID) {
                return 0;
            }

            @Override
            public REVLibError setSmartMotionMaxVelocity(double maxVel, int slotID) {
                return null;
            }

            @Override
            public REVLibError setSmartMotionMaxAccel(double maxAccel, int slotID) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setSmartMotionMinOutputVelocity(double minVel, int slotID) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setSmartMotionAllowedClosedLoopError(double allowedErr, int slotID) {
                return REVLibError.kOk;
            }

            @Override
            public REVLibError setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy accelStrategy, int slotID) {
                return REVLibError.kOk;
            }

            @Override
            public double getSmartMotionMaxVelocity(int slotID) {
                return 0;
            }

            @Override
            public double getSmartMotionMaxAccel(int slotID) {
                return 0;
            }

            @Override
            public double getSmartMotionMinOutputVelocity(int slotID) {
                return 0;
            }

            @Override
            public double getSmartMotionAllowedClosedLoopError(int slotID) {
                return 0;
            }

            @Override
            public SparkMaxPIDController.AccelStrategy getSmartMotionAccelStrategy(int slotID) {
                return SparkMaxPIDController.AccelStrategy.kTrapezoidal;
            }

            @Override
            public REVLibError setIMaxAccum(double iMaxAccum, int slotID) {
                return REVLibError.kOk;
            }

            @Override
            public double getIMaxAccum(int slotID) {
                return 0;
            }

            @Override
            public REVLibError setIAccum(double iAccum) {
                return REVLibError.kOk;
            }

            @Override
            public double getIAccum() {
                return 0;
            }

            @Override
            public REVLibError setFeedbackDevice(MotorFeedbackSensor sensor) {
                return REVLibError.kOk;
            }
        };
    }

    @Override
    public ISparkMaxLimitSwitch getIForwardLimitSwitch(SparkMaxLimitSwitch.Type switchType) {
        return new ISparkMaxLimitSwitch() {
            @Override
            public SparkMaxLimitSwitch.Type getType() {
                return SparkMaxLimitSwitch.Type.kNormallyClosed;
            }

            @Override
            public boolean isPressed() {
                return false;
            }

            @Override
            public REVLibError enableLimitSwitch(boolean enable) {
                return REVLibError.kOk;
            }

            @Override
            public boolean isLimitSwitchEnabled() {
                return false;
            }
        };
    }

    @Override
    public ISparkMaxLimitSwitch getIReverseLimitSwitch(SparkMaxLimitSwitch.Type switchType) {
        return new ISparkMaxLimitSwitch() {
            @Override
            public SparkMaxLimitSwitch.Type getType() {
                return SparkMaxLimitSwitch.Type.kNormallyClosed;
            }

            @Override
            public boolean isPressed() {
                return false;
            }

            @Override
            public REVLibError enableLimitSwitch(boolean enable) {
                return REVLibError.kOk;
            }

            @Override
            public boolean isLimitSwitchEnabled() {
                return false;
            }
        };
    }

    @Override
    public REVLibError setSmartCurrentLimit(int limit) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setSecondaryCurrentLimit(double limit) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setSecondaryCurrentLimit(double limit, int chopCycles) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setIdleMode(CANSparkMax.IdleMode mode) {
        return REVLibError.kOk;
    }

    @Override
    public CANSparkMax.IdleMode getIdleMode() {
        return CANSparkMax.IdleMode.kCoast;
    }

    @Override
    public REVLibError enableVoltageCompensation(double nominalVoltage) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError disableVoltageCompensation() {
        return REVLibError.kOk;
    }

    @Override
    public double getVoltageCompensationNominalVoltage() {
        return 0;
    }

    @Override
    public REVLibError setOpenLoopRampRate(double rate) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setClosedLoopRampRate(double rate) {
        return REVLibError.kOk;
    }

    @Override
    public double getOpenLoopRampRate() {
        return 0;
    }

    @Override
    public double getClosedLoopRampRate() {
        return 0;
    }

    @Override
    public REVLibError follow(CANSparkMax leader) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError follow(CANSparkMax leader, boolean invert) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError follow(CANSparkMax.ExternalFollower leader, int deviceID) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError follow(CANSparkMax.ExternalFollower leader, int deviceID, boolean invert) {
        return REVLibError.kOk;
    }

    @Override
    public boolean isFollower() {
        return false;
    }

    @Override
    public short getFaults() {
        return 0;
    }

    @Override
    public short getStickyFaults() {
        return 0;
    }

    @Override
    public boolean getFault(CANSparkMax.FaultID faultID) {
        return false;
    }

    @Override
    public boolean getStickyFault(CANSparkMax.FaultID faultID) {
        return false;
    }

    @Override
    public double getBusVoltage() {
        return 0;
    }

    @Override
    public double getAppliedOutput() {
        return 0;
    }

    @Override
    public double getOutputCurrent() {
        return 0;
    }

    @Override
    public double getMotorTemperature() {
        return 0;
    }

    @Override
    public REVLibError clearFaults() {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError burnFlash() {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setCANTimeout(int milliseconds) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError enableSoftLimit(CANSparkMax.SoftLimitDirection direction, boolean enable) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setSoftLimit(CANSparkMax.SoftLimitDirection direction, float limit) {
        return REVLibError.kOk;
    }

    @Override
    public double getSoftLimit(CANSparkMax.SoftLimitDirection direction) {
        return 0;
    }

    @Override
    public boolean isSoftLimitEnabled(CANSparkMax.SoftLimitDirection direction) {
        return false;
    }

    @Override
    public REVLibError getLastError() {
        return REVLibError.kOk;
    }

    @Override
    public int getFirmwareVersion() {
        return 0;
    }

    @Override
    public void setControlFramePeriodMs(int periodMs) {

    }

    @Override
    public String getFirmwareString() {
        return "ghost";
    }

    @Override
    public byte[] getSerialNumber() {
        return new byte[0];
    }

    @Override
    public int getDeviceId() {
        return 0;
    }

    @Override
    public CANSparkMaxLowLevel.MotorType getMotorType() {
        return CANSparkMaxLowLevel.MotorType.kBrushless;
    }

    @Override
    public REVLibError setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame frame, int periodMs) {
        return REVLibError.kOk;
    }

    @Override
    public void set(double speed) {

    }

    @Override
    public double get() {
        return 0;
    }

    @Override
    public void setInverted(boolean isInverted) {

    }

    @Override
    public boolean getInverted() {
        return false;
    }

    @Override
    public void disable() {

    }

    @Override
    public void stopMotor() {

    }
}
