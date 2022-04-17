package com.kennedyrobotics.hardware.motors.rev;

import com.revrobotics.*;

/**
 * Interfaces containing the non-deprecated methods of {@link com.revrobotics.CANSparkMax}
 */
public interface ICANSParkMax extends ICANSparkMaxLowLevel {

    /**
     * Returns an object for interfacing with the hall sensor integrated into a brushless motor, which
     * is connected to the front port of the SPARK MAX.
     *
     * <p>To access a quadrature encoder connected to the encoder pins or the front port of the SPARK
     * MAX, you must call the version of this method with EncoderType and countsPerRev parameters.
     *
     * @return An object for interfacing with the integrated encoder.
     */
    public RelativeEncoder getEncoder();

    /**
     * Returns an object for interfacing with the encoder connected to the encoder pins or front port
     * of the SPARK MAX.
     *
     * @param encoderType The encoder type for the motor: kHallEffect or kQuadrature
     * @param countsPerRev The counts per revolution of the encoder
     * @return An object for interfacing with an encoder
     */
    public RelativeEncoder getEncoder(SparkMaxRelativeEncoder.Type encoderType, int countsPerRev);

    /**
     * Returns an object for interfacing with a quadrature encoder connected to the alternate encoder
     * mode data port pins. These are defined as:
     *
     * <ul>
     *   <li>Pin 4 (Forward Limit Switch): Index
     *   <li>Pin 6 (Multi-function): Encoder A
     *   <li>Pin 8 (Reverse Limit Switch): Encoder B
     * </ul>
     *
     * <p>This call will disable support for the limit switch inputs.
     *
     * @param countsPerRev The counts per revolution of the encoder
     * @return An object for interfacing with a quadrature encoder connected to the alternate encoder
     *     mode data port pins
     */
    public RelativeEncoder getAlternateEncoder(int countsPerRev);

    /**
     * Returns an object for interfacing with a quadrature encoder connected to the alternate encoder
     * mode data port pins. These are defined as:
     *
     * <ul>
     *   <li>Pin 4 (Forward Limit Switch): Index
     *   <li>Pin 6 (Multi-function): Encoder A
     *   <li>Pin 8 (Reverse Limit Switch): Encoder B
     * </ul>
     *
     * <p>This call will disable support for the limit switch inputs.
     *
     * @param encoderType The encoder type for the motor: currently only kQuadrature
     * @param countsPerRev The counts per revolution of the encoder
     * @return An object for interfacing with a quadrature encoder connected to the alternate encoder
     *     mode data port pins
     */
    public RelativeEncoder getAlternateEncoder(
        SparkMaxAlternateEncoder.Type encoderType, int countsPerRev);

    /**
     * @param mode The mode of the analog sensor, either absolute or relative
     * @return An object for interfacing with a connected analog sensor.
     */
    public ISparkMaxAnalogSensor getIAnalog(SparkMaxAnalogSensor.Mode mode);

    /** @return An object for interfacing with the integrated PID controller. */
    public ISparkMaxPIDController getISparkPIDController();

    /**
     * Returns an object for interfacing with the forward limit switch connected to the appropriate
     * pins on the data port.
     *
     * <p>This call will disable support for the alternate encoder.
     *
     * @param switchType Whether the limit switch is normally open or normally closed.
     * @return An object for interfacing with the forward limit switch.
     */
    public ISparkMaxLimitSwitch getIForwardLimitSwitch(SparkMaxLimitSwitch.Type switchType);

    /**
     * Returns an object for interfacing with the reverse limit switch connected to the appropriate
     * pins on the data port.
     *
     * <p>This call will disable support for the alternate encoder.
     *
     * @param switchType Whether the limit switch is normally open or normally closed.
     * @return An object for interfacing with the reverse limit switch.
     */
    public ISparkMaxLimitSwitch getIReverseLimitSwitch(SparkMaxLimitSwitch.Type switchType);

    /**
     * Sets the current limit in Amps.
     *
     * <p>The motor controller will reduce the controller voltage output to avoid surpassing this
     * limit. This limit is enabled by default and used for brushless only. This limit is highly
     * recommended when using the NEO brushless motor.
     *
     * <p>The NEO Brushless Motor has a low internal resistance, which can mean large current spikes
     * that could be enough to cause damage to the motor and controller. This current limit provides a
     * smarter strategy to deal with high current draws and keep the motor and controller operating in
     * a safe region.
     *
     * @param limit The current limit in Amps.
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSmartCurrentLimit(int limit);

    /**
     * Sets the current limit in Amps.
     *
     * <p>The motor controller will reduce the controller voltage output to avoid surpassing this
     * limit. This limit is enabled by default and used for brushless only. This limit is highly
     * recommended when using the NEO brushless motor.
     *
     * <p>The NEO Brushless Motor has a low internal resistance, which can mean large current spikes
     * that could be enough to cause damage to the motor and controller. This current limit provides a
     * smarter strategy to deal with high current draws and keep the motor and controller operating in
     * a safe region.
     *
     * <p>The controller can also limit the current based on the RPM of the motor in a linear fashion
     * to help with controllability in closed loop control. For a response that is linear the entire
     * RPM range leave limit RPM at 0.
     *
     * @param stallLimit The current limit in Amps at 0 RPM.
     * @param freeLimit The current limit at free speed (5700RPM for NEO).
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit);

    /**
     * Sets the current limit in Amps.
     *
     * <p>The motor controller will reduce the controller voltage output to avoid surpassing this
     * limit. This limit is enabled by default and used for brushless only. This limit is highly
     * recommended when using the NEO brushless motor.
     *
     * <p>The NEO Brushless Motor has a low internal resistance, which can mean large current spikes
     * that could be enough to cause damage to the motor and controller. This current limit provides a
     * smarter strategy to deal with high current draws and keep the motor and controller operating in
     * a safe region.
     *
     * <p>The controller can also limit the current based on the RPM of the motor in a linear fashion
     * to help with controllability in closed loop control. For a response that is linear the entire
     * RPM range leave limit RPM at 0.
     *
     * @param stallLimit The current limit in Amps at 0 RPM.
     * @param freeLimit The current limit at free speed (5700RPM for NEO).
     * @param limitRPM RPM less than this value will be set to the stallLimit, RPM values greater than
     *     limitRPM will scale linearly to freeLimit
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM);

    /**
     * Sets the secondary current limit in Amps.
     *
     * <p>The motor controller will disable the output of the controller briefly if the current limit
     * is exceeded to reduce the current. This limit is a simplified 'on/off' controller. This limit
     * is enabled by default but is set higher than the default Smart Current Limit.
     *
     * <p>The time the controller is off after the current limit is reached is determined by the
     * parameter limitCycles, which is the number of PWM cycles (20kHz). The recommended value is the
     * default of 0 which is the minimum time and is part of a PWM cycle from when the over current is
     * detected. This allows the controller to regulate the current close to the limit value.
     *
     * <p>The total time is set by the equation <code>
     * t = (50us - t0) + 50us * limitCycles
     * t = total off time after over current
     * t0 = time from the start of the PWM cycle until over current is detected
     * </code>
     *
     * @param limit The current limit in Amps.
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSecondaryCurrentLimit(double limit);

    /**
     * Sets the secondary current limit in Amps.
     *
     * <p>The motor controller will disable the output of the controller briefly if the current limit
     * is exceeded to reduce the current. This limit is a simplified 'on/off' controller. This limit
     * is enabled by default but is set higher than the default Smart Current Limit.
     *
     * <p>The time the controller is off after the current limit is reached is determined by the
     * parameter limitCycles, which is the number of PWM cycles (20kHz). The recommended value is the
     * default of 0 which is the minimum time and is part of a PWM cycle from when the over current is
     * detected. This allows the controller to regulate the current close to the limit value.
     *
     * <p>The total time is set by the equation <code>
     * t = (50us - t0) + 50us * limitCycles
     * t = total off time after over current
     * t0 = time from the start of the PWM cycle until over current is detected
     * </code>
     *
     * @param limit The current limit in Amps.
     * @param chopCycles The number of additional PWM cycles to turn the driver off after overcurrent
     *     is detected.
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSecondaryCurrentLimit(double limit, int chopCycles);

    /**
     * Sets the idle mode setting for the SPARK MAX.
     *
     * @param mode Idle mode (coast or brake).
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setIdleMode(CANSparkMax.IdleMode mode);

    /**
     * Gets the idle mode setting for the SPARK MAX.
     *
     * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
     * non-blocking call and will return a cached value if the parameter is not returned by the
     * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
     *
     * @return IdleMode Idle mode setting
     */
    public CANSparkMax.IdleMode getIdleMode();

    /**
     * Sets the voltage compensation setting for all modes on the SPARK MAX and enables voltage
     * compensation.
     *
     * @param nominalVoltage Nominal voltage to compensate output to
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError enableVoltageCompensation(double nominalVoltage);

    /**
     * Disables the voltage compensation setting for all modes on the SPARK MAX.
     *
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError disableVoltageCompensation();

    /**
     * Get the configured voltage compensation nominal voltage value
     *
     * @return The nominal voltage for voltage compensation mode.
     */
    public double getVoltageCompensationNominalVoltage();

    /**
     * Sets the ramp rate for open loop control modes.
     *
     * <p>This is the maximum rate at which the motor controller's output is allowed to change.
     *
     * @param rate Time in seconds to go from 0 to full throttle.
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setOpenLoopRampRate(double rate);

    /**
     * Sets the ramp rate for closed loop control modes.
     *
     * <p>This is the maximum rate at which the motor controller's output is allowed to change.
     *
     * @param rate Time in seconds to go from 0 to full throttle.
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setClosedLoopRampRate(double rate);

    /**
     * Get the configured open loop ramp rate
     *
     * <p>This is the maximum rate at which the motor controller's output is allowed to change.
     *
     * @return ramp rate time in seconds to go from 0 to full throttle.
     */
    public double getOpenLoopRampRate();

    /**
     * Get the configured closed loop ramp rate
     *
     * <p>This is the maximum rate at which the motor controller's output is allowed to change.
     *
     * @return ramp rate time in seconds to go from 0 to full throttle.
     */
    public double getClosedLoopRampRate();

    /**
     * Causes this controller's output to mirror the provided leader.
     *
     * <p>Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
     *
     * <p>The motor will spin in the same direction as the leader. This can be changed by passing a
     * true constant after the leader parameter.
     *
     * <p>Following anything other than a CAN SPARK MAX is not officially supported.
     *
     * @param leader The motor controller to follow.
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError follow(final CANSparkMax leader);

    /**
     * Causes this controller's output to mirror the provided leader.
     *
     * <p>Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
     *
     * <p>Following anything other than a CAN SPARK MAX is not officially supported.
     *
     * @param leader The motor controller to follow.
     * @param invert Set the follower to output opposite of the leader
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError follow(final CANSparkMax leader, boolean invert);

    /**
     * Causes this controller's output to mirror the provided leader.
     *
     * <p>Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
     *
     * <p>The motor will spin in the same direction as the leader. This can be changed by passing a
     * true constant after the deviceID parameter.
     *
     * <p>Following anything other than a CAN SPARK MAX is not officially supported.
     *
     * @param leader The type of motor controller to follow (Talon SRX, SPARK MAX, etc.).
     * @param deviceID The CAN ID of the device to follow.
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError follow(CANSparkMax.ExternalFollower leader, int deviceID);

    /**
     * Causes this controller's output to mirror the provided leader.
     *
     * <p>Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
     *
     * <p>Following anything other than a CAN SPARK MAX is not officially supported.
     *
     * @param leader The type of motor controller to follow (Talon SRX, SPARK MAX, etc.).
     * @param deviceID The CAN ID of the device to follow.
     * @param invert Set the follower to output opposite of the leader
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError follow(CANSparkMax.ExternalFollower leader, int deviceID, boolean invert);

    /**
     * Returns whether the controller is following another controller
     *
     * @return True if this device is following another controller false otherwise
     */
    public boolean isFollower();

    /** @return All fault bits as a short */
    public short getFaults();

    /** @return All sticky fault bits as a short */
    public short getStickyFaults();

    /**
     * Get the value of a specific fault
     *
     * @param faultID The ID of the fault to retrive
     * @return True if the fault with the given ID occurred.
     */
    public boolean getFault(CANSparkMax.FaultID faultID);

    /**
     * Get the value of a specific sticky fault
     *
     * @param faultID The ID of the sticky fault to retrive
     * @return True if the sticky fault with the given ID occurred.
     */
    public boolean getStickyFault(CANSparkMax.FaultID faultID);

    /** @return The voltage fed into the motor controller. */
    public double getBusVoltage();

    /** @return The motor controller's applied output duty cycle. */
    public double getAppliedOutput();

    /** @return The motor controller's output current in Amps. */
    public double getOutputCurrent();

    /** @return The motor temperature in Celsius. */
    public double getMotorTemperature();

    /**
     * Clears all sticky faults.
     *
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError clearFaults();

    /**
     * Writes all settings to flash.
     *
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError burnFlash();

    /**
     * Sets timeout for sending CAN messages with SetParameter* and GetParameter* calls. These calls
     * will block for up to this amoutn of time before returning a timeout erro. A timeout of 0 will
     * make the SetParameter* calls non-blocking, and instead will check the response in a separate
     * thread. With this configuration, any error messages will appear on the drivestration but will
     * not be returned by the GetLastError() call.
     *
     * @param milliseconds The timeout in milliseconds.
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setCANTimeout(int milliseconds);

    /**
     * Enable soft limits
     *
     * @param direction the direction of motion to restrict
     * @param enable set true to enable soft limits
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError enableSoftLimit(CANSparkMax.SoftLimitDirection direction, boolean enable);

    /**
     * Set the soft limit based on position. The default unit is rotations, but will match the unit
     * scaling set by the user.
     *
     * <p>Note that this value is not scaled internally so care must be taken to make sure these units
     * match the desired conversion
     *
     * @param direction the direction of motion to restrict
     * @param limit position soft limit of the controller
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSoftLimit(CANSparkMax.SoftLimitDirection direction, float limit);

    /**
     * Get the soft limit setting in the controller
     *
     * @param direction the direction of motion to restrict
     * @return position soft limit setting of the controller
     */
    public double getSoftLimit(CANSparkMax.SoftLimitDirection direction);

    /**
     * @param direction The direction of the motion to restrict
     * @return true if the soft limit is enabled.
     */
    public boolean isSoftLimitEnabled(CANSparkMax.SoftLimitDirection direction);

    /**
     * All device errors are tracked on a per thread basis for all devices in that thread. This is
     * meant to be called immediately following another call that has the possibility of returning an
     * error to validate if an error has occurred.
     *
     * @return the last error that was generated.
     */
    public REVLibError getLastError();


}
