package com.kennedyrobotics.hardware.motors.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXPIDSetConfiguration;

public interface IVictorSPX extends IBaseMotorController {

    /**
     * Sets the appropriate output on the motor controller, depending on the mode.
     *
     * @param mode  The output mode to apply.
     *              In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
     *              In Velocity mode, output value is in position change / 100ms.
     *              In Position mode, output value is in encoder ticks or an analog value,
     *              depending on the sensor.
     *              In Follower mode, the output value is the integer device ID of the motor controller to duplicate.
     * @param value The setpoint value, as described above.
     *              <p>
     *              <p>
     *              Standard Driving Example:
     *              _victorLeft.set(ControlMode.PercentOutput, leftJoy);
     *              _victorRght.set(ControlMode.PercentOutput, rghtJoy);
     */
    public void set(VictorSPXControlMode mode, double value);

    /**
     * @param mode        Sets the appropriate output on the motor controller, depending on the mode.
     * @param demand0     The output value to apply.
     *                    such as advanced feed forward and/or auxiliary close-looping in firmware.
     *                    In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
     *                    In Velocity mode, output value is in position change / 100ms.
     *                    In Position mode, output value is in encoder ticks or an analog value,
     *                    depending on the sensor. See
     *                    In Follower mode, the output value is the integer device ID of the motor controller to
     *                    duplicate.
     * @param demand1Type The demand type for demand1.
     *                    Neutral: Ignore demand1 and apply no change to the demand0 output.
     *                    AuxPID: Use demand1 to set the target for the auxiliary PID 1.  Auxiliary
     *                    PID is always executed as standard Position PID control.
     *                    ArbitraryFeedForward: Use demand1 as an arbitrary additive value to the
     *                    demand0 output.  In PercentOutput the demand0 output is the motor output,
     *                    and in closed-loop modes the demand0 output is the output of PID0.
     * @param demand1     Supplmental output value.
     *                    AuxPID: Target position in Sensor Units
     *                    ArbitraryFeedForward: Percent Output between -1.0 and 1.0
     *                    <p>
     *                    <p>
     *                    Arcade Drive Example:
     *                    _victorLeft.set(ControlMode.PercentOutput, joyForward, DemandType.ArbitraryFeedForward, +joyTurn);
     *                    _victorRght.set(ControlMode.PercentOutput, joyForward, DemandType.ArbitraryFeedForward, -joyTurn);
     *                    <p>
     *                    Drive Straight Example:
     *                    Note: Selected Sensor Configuration is necessary for both PID0 and PID1.
     *                    _victorLeft.follow(_victorRght, FollwerType.AuxOutput1);
     *                    _victorRght.set(ControlMode.PercentOutput, joyForward, DemandType.AuxPID, desiredRobotHeading);
     *                    <p>
     *                    Drive Straight to a Distance Example:
     *                    Note: Other configurations (sensor selection, PID gains, etc.) need to be set.
     *                    _victorLeft.follow(_victorRght, FollwerType.AuxOutput1);
     *                    _victorRght.set(ControlMode.MotionMagic, targetDistance, DemandType.AuxPID, desiredRobotHeading);
     */
    public void set(VictorSPXControlMode mode, double demand0, DemandType demand1Type, double demand1);

    /**
     * Gets all PID set persistant settings.
     *
     * @param pid       Object with all of the PID set persistant settings
     * @param pidIdx    0 for Primary closed-loop. 1 for auxiliary closed-loop.
     * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
     *                  config success and report an error if it times out.
     *                  If zero, no blocking or checking is performed.
     */
    public void getPIDConfigs(VictorSPXPIDSetConfiguration pid, int pidIdx, int timeoutMs);

    /**
     * Gets all PID set persistant settings (overloaded so timeoutMs is 50 ms
     * and pidIdx is 0).
     *
     * @param pid Object with all of the PID set persistant settings
     */
    public void getPIDConfigs(VictorSPXPIDSetConfiguration pid);

    /**
     * Configures all persistent settings.
     *
     * @param allConfigs Object with all of the persistant settings
     * @param timeoutMs  Timeout value in ms. If nonzero, function will wait for
     *                   config success and report an error if it times out.
     *                   If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configAllSettings(VictorSPXConfiguration allConfigs, int timeoutMs);

    /**
     * Configures all persistent settings (overloaded so timeoutMs is 50 ms).
     *
     * @param allConfigs        Object with all of the persistant settings
     *
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configAllSettings(VictorSPXConfiguration allConfigs);

    /**
     * Gets all persistant settings.
     *
     * @param allConfigs        Object with all of the persistant settings
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
    public void getAllConfigs(VictorSPXConfiguration allConfigs, int timeoutMs);

    /**
     * Gets all persistant settings (overloaded so timeoutMs is 50 ms).
     *
     * @param allConfigs        Object with all of the persistant settings
     */
    public void getAllConfigs(VictorSPXConfiguration allConfigs);
}