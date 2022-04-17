package com.kennedyrobotics.hardware.motors.rev;

import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxLimitSwitch;

public interface ISparkMaxLimitSwitch {

    /**
     * Retrive the current configured type of limit switch.
     * @return type of limit switch
     */
    public SparkMaxLimitSwitch.Type getType();

    /**
     * Returns {@code true} if the limit switch is pressed, based on the selected polarity.
     *
     * <p>This method works even if the limit switch is not enabled for controller shutdown.
     *
     * @return {@code true} if the limit switch is pressed
     */
    public boolean isPressed();

    /**
     * Enables or disables controller shutdown based on the limit switch.
     *
     * @param enable Enable/disable motor shutdown based on the limit switch state. This does not
     *     affect the result of the get() command.
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError enableLimitSwitch(boolean enable);

    /** @return {@code true} if the limit switch is enabled */
    public boolean isLimitSwitchEnabled();
}
