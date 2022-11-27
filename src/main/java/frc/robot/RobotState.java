package frc.robot;

public class RobotState {
    // Types of transforms
    // - Static: The transform between reference frames does not change.
    // - Dynamic: The transform between reference frames change depending on the robot state. ei Turret angle, wheel odometry.

    // Transform name                        | Type
    // ------------------------------------- | ----
    // Field -> Field Starting Location      | Static transform (though does change depending on where the robot starts).
    // Field Starting Location -> DriveTrain | Dynamic transform
    // DriveTrain -> Turret                  | Static transform
    // Turret -> Limelight                   | Static transform
    // Limelight -> Hub                      | Dynamic transform
}
