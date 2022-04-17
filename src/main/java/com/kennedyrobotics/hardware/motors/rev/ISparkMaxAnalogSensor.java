package com.kennedyrobotics.hardware.motors.rev;

import com.revrobotics.AnalogInput;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAnalogSensor;

/**
 * Interface containing the non-deprecated methods of {@link com.revrobotics.SparkMaxAnalogSensor}.
 */
public interface ISparkMaxAnalogSensor extends AnalogInput, MotorFeedbackSensor {

    /**
     * Retrieve the current mode the analog sensor is configured with.
     * @return Current mode of the Analog sensor.
     */
    public SparkMaxAnalogSensor.Mode getMode();

    /**
     * Get the velocity of the sensor. Returns value in the native units of 'volts per second' by
     * default, and can be changed by a scale factor using setVelocityConversionFactor().
     *
     * @return Velocity of the sensor in volts per second
     */
    public double getVelocity();

    /**
     * Set the conversion factor for the velocity of the analog sensor. By default, revolutions per
     * volt second is 1. Changing the velocity conversion factor will also change the velocity units.
     *
     * @param factor The conversion factor which will be multiplied by volts per second
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setVelocityConversionFactor(double factor);

    /**
     * Get the current conversion factor for the velocity of the analog sensor.
     *
     * @return Analog velocity conversion factor
     */
    public double getVelocityConversionFactor();
}
