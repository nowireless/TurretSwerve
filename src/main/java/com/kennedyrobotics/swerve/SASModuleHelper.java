package com.kennedyrobotics.swerve;

import com.kennedyrobotics.swerve.misc.NullAbsoluteEncoderConfiguration;
import com.kennedyrobotics.swerve.misc.NullAbsoluteEncoderFactoryBuilder;
import com.kennedyrobotics.swerve.rev.Neo550SteerConfiguration;
import com.kennedyrobotics.swerve.rev.Neo550SteerControllerFactoryBuilder;
import com.kennedyrobotics.swerve.rev.NeoDriveControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SASModuleHelper {

    private static DriveControllerFactory<?, Integer> getNeoDriveFactory(SASModuleConfiguration configuration) {
        return new NeoDriveControllerFactoryBuilder()
            .withVoltageCompensation(configuration.getNominalVoltage())
            .withCurrentLimit(configuration.getDriveCurrentLimit())
            .withRampRate(0.1) // This prevents the module from stuttering
            .build();
    }

    private static SteerControllerFactory<?, Neo550SteerConfiguration<NullAbsoluteEncoderConfiguration>> getNeo550SteerFactory(SASModuleConfiguration configuration, Rotation2d moduleOffset) {
        return new Neo550SteerControllerFactoryBuilder()
            .withVoltageCompensation(configuration.getNominalVoltage())
            .withPidConstants(15, 0, 0) // TODO add to SASModuleConfiguration
            .withCurrentLimit(configuration.getSteerCurrentLimit())
            .withRampRate(0.1) // This prevents the module from stuttering, TODO add to SASModuleConfiguration
            .withModuleOffset(moduleOffset)
            .build(new NullAbsoluteEncoderFactoryBuilder().build());
    }

    /**
     *
     * @param container
     * @param configuration
     * @param gearRatio
     * @param driveMotorPort
     * @param steerMotorPort
     * @param steerOffsetDegrees
     * @return
     */
    public static SwerveModule createV2(
        ShuffleboardLayout container,
        SASModuleConfiguration configuration,
        GearRatio gearRatio,
        int driveMotorPort,
        int steerMotorPort,
        Rotation2d steerOffsetDegrees
    ) {
        return new SwerveModuleFactory<> (
            gearRatio.getConfiguration(),
            getNeoDriveFactory(configuration),
            getNeo550SteerFactory(configuration, steerOffsetDegrees)
        ).create(
            container,
            driveMotorPort,
            new Neo550SteerConfiguration<>(
                steerMotorPort,
                new NullAbsoluteEncoderConfiguration()
            )
        );
    }

    /**
     * TODO
     */
    public enum GearRatio {
        V1(SASModuleConfigurations.V1),
        V2(SASModuleConfigurations.V2);

        private final ModuleConfiguration configuration;

        GearRatio(ModuleConfiguration configuration) {
            this.configuration = configuration;
        }

        public ModuleConfiguration getConfiguration() {
            return configuration;
        }

    }
}
