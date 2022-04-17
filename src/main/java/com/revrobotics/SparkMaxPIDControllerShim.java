package com.revrobotics;

import com.kennedyrobotics.hardware.motors.rev.ISparkMaxPIDController;

/**
 * Shim to make {@link SparkMaxPIDController} implement the {@link ISparkMaxPIDController} interface.
 *
 * This needs to be in the com.revrobotics package due to the construct of SparkMaxPIDController being package
 * private.
 */
public class SparkMaxPIDControllerShim extends SparkMaxPIDController implements ISparkMaxPIDController {

    public SparkMaxPIDControllerShim(CANSparkMax device) {
        super(device);
    }
}
