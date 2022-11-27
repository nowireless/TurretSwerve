package com.kennedyrobotics.swerve.misc;

import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;


/**
 * Use with Steer motors that do not require an external absolute encoder, such as a
 * Swerve and Steer module with a absolute encoder directly connected to a motor controller
 */
public class NullAbsoluteEncoderFactoryBuilder {

    public AbsoluteEncoderFactory<NullAbsoluteEncoderConfiguration> build() {
        return configuration -> new EncoderImplementation();
    }

    /**
     * TODO
     */
    private static class EncoderImplementation implements AbsoluteEncoder {

        @Override
        public double getAbsoluteAngle() {
            return Double.NaN;
        }
    }
}
