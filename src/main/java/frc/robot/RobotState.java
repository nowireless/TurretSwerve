package frc.robot;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

public class RobotState {

    private static final double kObservationBufferTime = 2.0;



    // Frames
    // | Frame Name    | Description
    // | ------------- | -----------
    // | Field frame   | Origin is where the robot is turned on.
    // | Vehicle frame | Origin is the center of the robot wheelbase, facing forwards
    // | Turret frame  | Origin is the center of the turret.
    // | Camera frame  | Origin is the center of the Limelight relative to the turret.
    // | Target frame  | Origin is the center of the vision target, facing outwards along the normal.


    // Types of transforms
    // - Static: The transform between reference frames does not change.
    // - Dynamic: The transform between reference frames change depending on the robot state. ei Turret angle, wheel odometry.

    // | Transform name                | Description
    // | ----------------------------- | -----------
    // |                               | This is tracked over time by integrating encoder and gyro measurements.
    // | Field frame -> Vehicle frame  | It will inevitably drift, but is usually accurate over short time periods.
    // |                               |
    // | Vehicle frame -> Turret frame | Rotation measured by the turret encoder; translation is constant.
    // | Turret frame -> Camera frame  | This is a constant (per camera).
    // | Camera frame -> Target frame  | Measured by the vision system.

    // FPGATimestamp -> Pose2d or Rotation2d
    private TimeInterpolatableBuffer<Pose2d> m_fieldToVehicle = TimeInterpolatableBuffer.createBuffer(kObservationBufferTime);
    private TimeInterpolatableBuffer<Pose2d> m_vehicleToTurret = TimeInterpolatableBuffer.createBuffer(kObservationBufferTime);

    public synchronized void reset() {
        // TODO put stuff here
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */

    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return m_fieldToVehicle.getSample(timestamp);
    }

    public synchronized Transform2d getVehicleToTurret(double timestamp) {
        Pose2d vehicleToTurret =  m_vehicleToTurret.getSample(timestamp);
        return new Transform2d(
            vehicleToTurret.getTranslation(),
            vehicleToTurret.getRotation()
        );
    }

    public synchronized Pose2d getFieldToTurret(double timestamp) {
        // From 254
        // return new Pose2d(translation_.translateBy(other.translation_.rotateBy(rotation_)),
        //    rotation_.rotateBy(other.rotation_));

        return getFieldToVehicle(timestamp).transformBy(getVehicleToTurret(timestamp));
    }

    public synchronized getLastestFieldToVehicle() {
        return m_fieldToVehicle.
    }


}
