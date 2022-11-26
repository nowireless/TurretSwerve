// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.List;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */


  // Helpers
  SwerveDriveOdometry odometry;

  // Hardware
  private final Pigeon2 imu;
  private final SwerveModule frontLeft;
  private final SwerveModule rearLeft;
  private final SwerveModule frontRight;
  private final SwerveModule rearRight;
  private final List<SwerveModule> modules;

  public Drivetrain() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    //
    // IMU setup
    //
    imu = new Pigeon2(DriveConstants.kPigeonID, DriveConstants.kPigeonCANBus.busName);

    //
    // Swerve Modules
    //
    frontLeft = new SwerveModule(
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        ModuleConstants.kFrontLeftConfig
    );
    rearLeft = new SwerveModule(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        ModuleConstants.kRearLeftConfig
    );
    frontRight = new SwerveModule(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        ModuleConstants.kFrontRightConfig
    );
    rearRight = new SwerveModule(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        ModuleConstants.kRearRightConfig
    );

    modules = Arrays.asList(
      frontLeft,
      frontRight,
      rearLeft,
      rearRight
    );

    // Odometry class for tracking robot pose
    odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getHeading());

  }

  public Rotation2d getHeading() {
    // TODO make sure the angles here make sense
    return Rotation2d.fromDegrees(imu.getYaw());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    // TODO make sure the angles here make sense

    double[] xyzDps = new double[3];
    imu.getRawGyro(xyzDps);
    return xyzDps[2];
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getHeading());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds;

    if (fieldRelative) {
      // Field centric control
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading());
    } else {
      // Robot centric control
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    // Send it!
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }

  /**
   * Stop the drive train
   */
  public void stop() {
    // TODO want a specail stop mode?
    drive(0, 0, 0, false);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kMaxVelocityMetersPerSecond);

    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    imu.setYaw(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the odometry in the periodic block
    odometry.update(
        getHeading(),
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState()
    );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
