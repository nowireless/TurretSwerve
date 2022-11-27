package com.kennedyrobotics.swerve;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import edu.wpi.first.math.util.Units;

public class SASModuleConfigurations {

    public static final ModuleConfiguration V1 = new ModuleConfiguration(
        Units.inchesToMeters(4),
        (12.0 / 40.0) * (20.0/40.0),
        true,
        (1.0/71.0) * (48.0/40.0),
        true
    );

    public static final ModuleConfiguration V2 = new ModuleConfiguration(
        Units.inchesToMeters(4),
        (12.0 / 40.0) * (20.0/40.0),
        true,
        (1.0 / 2.0) * (1.0/45.0), // Belt 2:1 -> Versa Planetary 1:45
        true
    );

    private SASModuleConfigurations() {}
}
