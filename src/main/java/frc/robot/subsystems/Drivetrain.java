// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kennedyrobotics.swerve.SASModuleConfiguration;
import com.kennedyrobotics.swerve.SASModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
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
  private final SwerveDriveOdometry m_odometry;
  private final Field2d m_field = new Field2d();

  // Hardware
  private final Pigeon2 m_imu;
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_rearRight;
  private final List<SwerveModule> m_modules;



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
    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    SASModuleConfiguration moduleConfiguration = new SASModuleConfiguration();
    moduleConfiguration.setNominalVoltage(DriveConstants.kDriveVoltageCompensation);

    m_frontLeft = SASModuleHelper.createV2(
        // Push module information to shuffle boards
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 6)
            .withPosition(0, 0),
        moduleConfiguration,
        SASModuleHelper.GearRatio.V2,
        DriveConstants.kFrontLeftMotorDriveID,
        DriveConstants.kFrontLeftMotorSteerID,
        DriveConstants.kFrontLeftOffset
    );

    m_rearLeft = SASModuleHelper.createV2(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 6)
            .withPosition(4, 0),
        moduleConfiguration,
        SASModuleHelper.GearRatio.V2,
        DriveConstants.kRearLeftMotorDriveID,
        DriveConstants.kRearLeftMotorSteerID,
        DriveConstants.kRearLeftOffset
    );
    m_frontRight = SASModuleHelper.createV2(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 6)
            .withPosition(2, 0),
        moduleConfiguration,
        SASModuleHelper.GearRatio.V2,
        DriveConstants.kFrontRightMotorDriveID,
        DriveConstants.kFrontRightMotorSteerID,
        DriveConstants.kFrontRightOffset
    );
    m_rearRight = SASModuleHelper.createV2(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 6)
            .withPosition(6, 0),
        moduleConfiguration,
        SASModuleHelper.GearRatio.V2,
        DriveConstants.kRearRightMotorDriveID,
        DriveConstants.kRearRightMotorSteerID,
        DriveConstants.kRearRightOffset
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
    Rotation2d heading = Rotation2d.fromDegrees(m_imu.getYaw());

    if (DriveConstants.kPigeonUpsideDown) {
      heading = heading.unaryMinus();
    }

    return heading;
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
    // Ensure all module states have achievable values
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxDriveVelocityMetersPerSecond);

    // Optimize swerve module states. Prevent the swerve modules from moving farther then 90 degrees. If the direction
    // of the motor can be inverted
    desiredStates[0] = SwerveModuleState.optimize(desiredStates[0], new Rotation2d(m_frontLeft.getSteerAngle()));
    desiredStates[1] = SwerveModuleState.optimize(desiredStates[1], new Rotation2d(m_frontRight.getSteerAngle()));
    desiredStates[2] = SwerveModuleState.optimize(desiredStates[2], new Rotation2d(m_rearLeft.getSteerAngle()));
    desiredStates[3] = SwerveModuleState.optimize(desiredStates[3], new Rotation2d(m_rearRight.getSteerAngle()));

    // Send it!
    m_frontLeft.set(
        desiredStates[0].speedMetersPerSecond / DriveConstants.kMaxDriveVelocityMetersPerSecond * 12,
        desiredStates[0].angle.getRadians()
    );

    m_frontRight.set(
        desiredStates[1].speedMetersPerSecond / DriveConstants.kMaxDriveVelocityMetersPerSecond * 12,
        desiredStates[1].angle.getRadians()
    );

    m_rearLeft.set(
        desiredStates[2].speedMetersPerSecond / DriveConstants.kMaxDriveVelocityMetersPerSecond * 12,
        desiredStates[2].angle.getRadians()
    );

    m_rearRight.set(
        desiredStates[3].speedMetersPerSecond / DriveConstants.kMaxDriveVelocityMetersPerSecond * 12,
        desiredStates[3].angle.getRadians()
    );
  }

  public SwerveModuleState[] getModuleStates() {
    // Note the order of modules needs to match the order provided to DriveConstants.kDriveKinematics
    return new SwerveModuleState[]{
        new SwerveModuleState(m_frontLeft.getDriveVelocity(), new Rotation2d(m_frontLeft.getSteerAngle())),
        new SwerveModuleState(m_frontRight.getDriveVelocity(), new Rotation2d(m_frontRight.getSteerAngle())),
        new SwerveModuleState(m_rearLeft.getDriveVelocity(), new Rotation2d(m_rearLeft.getSteerAngle())),
        new SwerveModuleState(m_rearRight.getDriveVelocity(), new Rotation2d(m_rearRight.getSteerAngle()))
    };
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_imu.setYaw(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the odometry in the periodic block
    SwerveModuleState[] moduleStates = getModuleStates();
    m_odometry.update(
        getHeading(),
        moduleStates[0],
        moduleStates[1],
        moduleStates[2],
        moduleStates[3]
    );

    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
