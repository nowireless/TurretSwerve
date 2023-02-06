package com.kennedyrobotics.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkMaxJNI;

public class CANSparkMaxEx extends CANSparkMax {

    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceId The device ID.
     * @param type     The motor type connected to the controller. Brushless motor wires must be connected
     *                 to their matching colors and the hall sensor must be plugged in. Brushed motors must be
     */
    public CANSparkMaxEx(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    /**
     * Set the number of samples in the average for velocity readings. This
     * can be any value from 1 to 64.
     *
     * When the SparkMax controller is in Brushless mode, this
     * will not change any behavior.
     *
     * @param depth The average sampling depth between 1 and 64 (default)
     *
     * @return REVLibError::kOk if successful
     */
    public REVLibError setAnalogAverageDepth(int depth) {
        throwIfClosed();
        return REVLibError.fromInt(
            CANSparkMaxJNI.c_SparkMax_SetAnalogAverageDepth(sparkMaxHandle, depth)
        );
    }

    /**
     *
     * @return
     */
    public int getAnalogAverageDepth() {
        return CANSparkMaxJNI.c_SparkMax_GetAnalogAverageDepth(sparkMaxHandle);
    }

    /**
     * Set the measurement period for velocity readings.
     *
     * The basic formula to calculate velocity is change in position / change in
     * time. This parameter sets the change in time for measurement.
     *
     * @param period_ms Measurement period in milliseconds. This number may be
     * between 1 and 100 (default).
     *
     * @return REVLibError::kOk if successful
     */
    public REVLibError setAnalogMeasurementPeriod(int samples) {
        throwIfClosed();
        return REVLibError.fromInt(
            CANSparkMaxJNI.c_SparkMax_SetAnalogMeasurementPeriod(sparkMaxHandle, samples)
        );
    }

    /**
     *
     * @return
     */
    public int getAnalogMeasurementPeriod() {
        return CANSparkMaxJNI.c_SparkMax_GetAnalogMeasurementPeriod(sparkMaxHandle);
    }


}
