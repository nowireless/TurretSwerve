// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kennedyrobotics.swerve.SASModuleHelper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public enum CANBus {
        kRoboRIO("rio"),
        kCANivore("canivore");

        public final String busName;

        private CANBus(String busName) {
            this.busName = busName;
        }
    }

    public static class ModuleConstants {
        //
        // Global module configuration
        //
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

        public static final double kDriveGearReduction = SASModuleHelper.GearRatio.V2.getConfiguration().getDriveReduction();

        public static final double kDriveVoltageCompensation = 10;


        //  FreeSpeed Radians   1 Rotation                     kWheelDiameter Meters   Voltage Nominal   FreeSpeed * kGearReduction * kWheelDiameter Meters
        //  ----------------- * ----------- * kGearReduction * --------------------- * --------------- = --------------------------------------------------
        //  1 Second            2PI Radians                    1 Rotation              Voltage Max       2PI Second
        public static final double kMaxDriveVelocityMetersPerSecond = DCMotor.getNEO(1).freeSpeedRadPerSec / (2*Math.PI) * kDriveGearReduction * kWheelDiameterMeters * (kDriveVoltageCompensation/12.0);

        //
        // Individual module configuration
        //
        public static final int kFrontLeftMotorDriveID = 10;
        public static final int kFrontLeftMotorSteerID = 11;
        public static final int kFrontLeftEncoderID = 11;
        public static final Rotation2d kFrontLeftOffset = Rotation2d.fromDegrees(-64);

        public static final int kRearLeftMotorDriveID = 12;
        public static final int kRearLeftMotorSteerID = 13;
        public static final int kRearLeftEncoderID = 13;
        public static final Rotation2d kRearLeftOffset = Rotation2d.fromDegrees(70);

        public static final int kFrontRightMotorDriveID = 14;
        public static final int kFrontRightMotorSteerID = 15;
        public static final int kFrontRightEncoderID = 15;
        public static final Rotation2d kFrontRightOffset = Rotation2d.fromDegrees(-139);

        public static final int kRearRightMotorDriveID = 16;
        public static final int kRearRightMotorSteerID = 17;
        public static final int kRearRightEncoderID = 17;
        public static final Rotation2d kRearRightOffset = Rotation2d.fromDegrees(-65);
    }

    public static class DriveConstants {
        //
        // Physical constants
        //

        // Distance between centers of the front and rear wheels on robot
        public static final double kWheelBaseLengthMeters = Units.inchesToMeters(21.5);

        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidthMeters = Units.inchesToMeters(26);

        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         *
         * Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
         */
        public static final double kMaxAngularVelocityRadiansPerSecond = ModuleConstants.kMaxDriveVelocityMetersPerSecond /
            Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseLengthMeters / 2.0);

        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBaseLengthMeters / 2, kTrackWidthMeters / 2),
                new Translation2d(kWheelBaseLengthMeters / 2, -kTrackWidthMeters / 2),
                new Translation2d(-kWheelBaseLengthMeters / 2, kTrackWidthMeters / 2),
                new Translation2d(-kWheelBaseLengthMeters / 2, -kTrackWidthMeters / 2));

        //
        // CAN BUS IDS
        //
        public static final int kPigeonID = 0;
        public static final CANBus kPigeonCANBus = CANBus.kCANivore;
        public static final boolean kPigeonUpsideDown = false;
    }

    public static final class AutoConstants {
        // TODO fill in

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
