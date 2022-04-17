package com.kennedyrobotics.hardware.motors.ctre;

import com.ctre.phoenix.ErrorCode;

public interface ITalonFXSensorCollection {

    /**
     * Get the IntegratedSensor position of the Talon FX, regardless of whether
     * it is actually being used for feedback.  The units are 2048 per rotation.
     * Note : Future versions of software may support scaling features (rotations, radians, degrees, etc) depending on the configuration.
     * <p>
     * This method relies on the Status 21 message, which has a default period of 240ms. For more
     * information, see: https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html
     *
     * @return the IntegratedSensor position.
     */
    public double getIntegratedSensorPosition();

    /**
     * Get the IntegratedSensor absolute position of the Talon FX, regardless of whether
     * it is actually being used for feedback.  This will be within one rotation (2048 units).
     * The signage and range will depend on the configuration.
     * Note : Future versions of software may support scaling features (rotations, radians, degrees, etc) depending on the configuration.
     * <p>
     * This method relies on the Status 21 message, which has a default period of 240ms. For more
     * information, see: https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html
     *
     * @return the IntegratedSensor absolute position.
     */
    public double getIntegratedSensorAbsolutePosition();

    /**
     * Get the IntegratedSensor velocity of the Talon FX, regardless of whether
     * it is actually being used for feedback.
     * One unit represents one position unit per 100ms (2048 position units per 100ms).
     * The signage and range will depend on the configuration.
     * Note : Future versions of software may support scaling features (rotations, radians, degrees, etc) depending on the configuration.
     * <p>
     * This method relies on the Status 21 message, which has a default period of 240ms. For more
     * information, see: https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html
     *
     * @return the IntegratedSensor velocity.
     */
    public double getIntegratedSensorVelocity();

    /**
     * Set the IntegratedSensor reported position.  Typically this is used to "zero" the
     * sensor. This only works with IntegratedSensor.  To set the selected sensor position
     * regardless of what type it is, see SetSelectedSensorPosition in the motor controller class.
     *
     * @param newPosition The position value to apply to the sensor.
     * @param timeoutMs   Timeout value in ms. If nonzero, function will wait for
     *                    config success and report an error if it times out.
     *                    If zero, no blocking or checking is performed.
     * @return error code.
     */
    public ErrorCode setIntegratedSensorPosition(double newPosition,
                                                 int timeoutMs);

    /**
     * Set the IntegratedSensor reported position based on the absolute position.
     * This can also be done automatically on power boot depending on configuration.
     *
     * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
     *                  config success and report an error if it times out.
     *                  If zero, no blocking or checking is performed.
     * @return error code.
     */
    public ErrorCode setIntegratedSensorPositionToAbsolute(int timeoutMs);

    /**
     * Is forward limit switch closed.
     * <p>
     * This method relies on the Status 1 message, which has a default period of 10ms. For more
     * information, see: https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html
     *
     * @return '1' iff forward limit switch is closed, 0 iff switch is open. This function works
     * regardless if limit switch feature is enabled.  Remote limit features do not impact this routine.
     */
    public int isFwdLimitSwitchClosed();

    /**
     * Is reverse limit switch closed.
     * <p>
     * This method relies on the Status 1 message, which has a default period of 10ms. For more
     * information, see: https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html
     *
     * @return '1' iff reverse limit switch is closed, 0 iff switch is open. This function works
     * regardless if limit switch feature is enabled.  Remote limit features do not impact this routine.
     */
    public int isRevLimitSwitchClosed();
}
