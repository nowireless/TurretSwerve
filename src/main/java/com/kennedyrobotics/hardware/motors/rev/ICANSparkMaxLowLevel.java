package com.kennedyrobotics.hardware.motors.rev;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Interface containing the non-deprecated methods of {@link com.revrobotics.CANSparkMaxLowLevel}
 */
public interface ICANSparkMaxLowLevel extends MotorController {

    /**
     * Get the firmware version of the SPARK MAX.
     *
     * @return uint32_t Firmware version integer. Value is represented as 4 bytes, Major.Minor.Build
     *     H.Build L
     */
    public int getFirmwareVersion();

    /**
     * Set the control frame send period for the native CAN Send thread.
     *
     * @param periodMs The send period in milliseconds between 1ms and 100ms or set to 0 to disable
     *     periodic sends. Note this is not updated until the next call to Set() or SetReference().
     */
    public void setControlFramePeriodMs(int periodMs);

    /**
     * Get the firmware version of the SPARK MAX as a string.
     *
     * @return std::string Human readable firmware version string
     */
    public String getFirmwareString();

    /**
     * Get the unique serial number of the SPARK MAX. Not currently available.
     *
     * @return byte[] Vector of bytes representig the unique serial number
     */
    public byte[] getSerialNumber();

    /**
     * Get the configured Device ID of the SPARK MAX.
     *
     * @return int device ID
     */
    public int getDeviceId();

    /**
     * Get the motor type setting for the SPARK MAX
     *
     * @return MotorType Motor type setting
     */
    public CANSparkMaxLowLevel.MotorType getMotorType();

    /**
     * Set the rate of transmission for periodic frames from the SPARK MAX
     *
     * <p>Each motor controller sends back three status frames with different data at set rates. Use
     * this function to change the default rates.
     *
     * <p>Defaults: Status0 - 10ms Status1 - 20ms Status2 - 50ms
     *
     * <p>This value is not stored in the FLASH after calling burnFlash() and is reset on powerup.
     *
     * <p>Refer to the SPARK MAX reference manual on details for how and when to configure this
     * parameter.
     *
     * @param frame Which type of periodic frame to change the period of
     * @param periodMs The rate the controller sends the frame to the controller.
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame frame, int periodMs);
}
