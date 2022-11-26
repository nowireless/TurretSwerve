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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.List;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */


  // Helpers
  SwerveDriveOdometry m_odometry;

  // Hardware
  private final Pigeon2 m_imu;
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_rearRight;
  private final List<SwerveModule> m_modules;

  // TODO move into a locationization/robot state class?
  private final Field2d m_field = new Field2d();


  public Drivetrain() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    //
    // IMU setup
    //
    m_imu = new Pigeon2(DriveConstants.kPigeonID, DriveConstants.kPigeonCANBus.busName);
    var imuShuffleboard = tab.getLayout("IMU", BuiltInLayouts.kList)
        .withSize(2,2)
        .withPosition(8, 0);
    imuShuffleboard.addNumber("Heading", ()->getHeading().getDegrees());
    imuShuffleboard.addNumber("Temperature", ()-> m_imu.getTemp() * 9.0/5.0+32.0);
    imuShuffleboard.addNumber("Uptime", m_imu::getUpTime);


    //
    // Swerve Modules
    //
    m_frontLeft = new SwerveModule(
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 6)
            .withPosition(0, 0),
        ModuleConstants.kFrontLeftConfig
    );
    m_rearLeft = new SwerveModule(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 6)
            .withPosition(4, 0),
        ModuleConstants.kRearLeftConfig
    );
    m_frontRight = new SwerveModule(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 6)
            .withPosition(2, 0),
        ModuleConstants.kFrontRightConfig
    );
    m_rearRight = new SwerveModule(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 6)
            .withPosition(6, 0),
        ModuleConstants.kRearRightConfig
    );

    m_modules = Arrays.asList(
        m_frontLeft,
        m_frontRight,
        m_rearLeft,
        m_rearRight
    );

    // Odometry class for tracking robot pose
    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getHeading());

    tab.add("Field", m_field)
        .withSize(5, 4)
        .withPosition(8,2)
        .withWidget(BuiltInWidgets.kField);

    var odometryTab = tab.getLayout("Odometry", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(10,0);

    odometryTab.addNumber("X (inches)", () -> Units.metersToInches(m_odometry.getPoseMeters().getX()));
    odometryTab.addNumber("Y (inches)", () -> Units.metersToInches(m_odometry.getPoseMeters().getY()));
    odometryTab.addNumber("Theta (degrees)", () -> m_odometry.getPoseMeters().getRotation().getDegrees());
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_imu.getYaw());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    // TODO make sure the angles here make sense

    double[] xyzDps = new double[3];
    m_imu.getRawGyro(xyzDps);
    return xyzDps[2];
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getHeading());
  }

  /**
   * Method to drive the robot using joystick info. All speeds are in meters per second.
   *
   * For more information, about coordinate systems see:
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
   *
   * @param xSpeed Speed of the robot in the x direction (forward). Positive values move forward.
   * @param ySpeed Speed of the robot in the y direction (sideways). Positive values move left
   * @param rot Angular rate of the robot. Positive values rotate counter-clockwise.
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
    drive(0, 0, 0, false);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kMaxDriveVelocityMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_imu.setYaw(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the odometry in the periodic block
    m_odometry.update(
        getHeading(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    );

    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
