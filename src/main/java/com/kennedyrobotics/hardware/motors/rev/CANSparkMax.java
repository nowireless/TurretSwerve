package com.kennedyrobotics.hardware.motors.rev;

import com.revrobotics.*;

public class CANSparkMax extends com.revrobotics.CANSparkMax implements ICANSParkMax {

    private ISparkMaxPIDController pidController;
    private final Object pidControllerLock = new Object();

    private ISparkMaxAnalogSensor analogSensor;
    private final Object analogSensorLock = new Object();

    private ISparkMaxLimitSwitch forwardLimitSwitch;
    private final Object forwardLimitSwitchLock = new Object();

    private ISparkMaxLimitSwitch reverseLimitSwitch;
    private final Object reverseLimitSwitchLock = new Object();

    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceId The device ID.
     * @param type     The motor type connected to the controller. Brushless motor wires must be connected
     *                 to their matching colors and the hall sensor must be plugged in. Brushed motors must be
     */
    public CANSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    @Override
    public ISparkMaxAnalogSensor getIAnalog(SparkMaxAnalogSensor.Mode mode) {
        throwIfClosed();
        synchronized (analogSensorLock) {
            if (analogSensor == null) {
                analogSensor = new SparkMaxAnalogSensorShim(this, mode);
            } else {
                if (analogSensor.getMode() != mode) {
                    throw new IllegalStateException(
                        "The analog sensor connected to this SPARK MAX has already been configured with mode "
                            + analogSensor.getMode());
                }
                // The user isn't trying to change their analog sensor settings mid-program,
                // so we're all set
            }
            return analogSensor;
        }
    }

    @Override
    public ISparkMaxPIDController getISparkPIDController() {
        throwIfClosed();
        synchronized (pidControllerLock) {
            if (pidController == null) {
                pidController = new SparkMaxPIDControllerShim(this);
            }
            return pidController;
        }
    }

    @Override
    public ISparkMaxLimitSwitch getIForwardLimitSwitch(SparkMaxLimitSwitch.Type switchType) {
        throwIfClosed();
        synchronized (forwardLimitSwitchLock) {
            if (forwardLimitSwitch == null) {
                forwardLimitSwitch = SparkMaxLimitSwitchShim.makeForwardLimitSwitch(this, switchType);
            } else {
                if (forwardLimitSwitch.getType() != switchType) {
                    throw new IllegalStateException(
                        "The forward limit switch on this SPARK MAX has already been configured with the"
                            + forwardLimitSwitch.getType()
                            + " switch type");
                }
            }
            return forwardLimitSwitch;
        }
    }

    @Override
    public ISparkMaxLimitSwitch getIReverseLimitSwitch(SparkMaxLimitSwitch.Type switchType) {
        throwIfClosed();
        synchronized (reverseLimitSwitchLock) {
            if (reverseLimitSwitch == null) {
                reverseLimitSwitch = SparkMaxLimitSwitchShim.makeReverseLimitSwitch(this, switchType);
            } else {
                if (reverseLimitSwitch.getType() != switchType) {
                    throw new IllegalStateException(
                        "The reverse limit switch on this SPARK MAX has already been configured with the "
                            + reverseLimitSwitch.getType()
                            + " switch type");
                }
            }
            return reverseLimitSwitch;
        }
    }

    // TODO/HACK I think since since CANSparkMax defines a final in the follow method args, they don't
    // implement the ICANSParkMax interface.

    @Override
    public REVLibError follow(CANSparkMax leader) {
        return super.follow(leader);
    }

    @Override
    public REVLibError follow(CANSparkMax leader, boolean invert) {
        return super.follow(leader, invert);
    }
}
