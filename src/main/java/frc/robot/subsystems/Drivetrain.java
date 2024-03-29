// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kennedyrobotics.swerve.SASModuleConfiguration;
import com.kennedyrobotics.swerve.SASModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
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
  private final SwerveDrivePoseEstimator m_poseEstimator;
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
    // Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
    // below are robot specific, and should be tuned.
    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getHeading(),
      getModulePositions(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );

    var driveConstants = tab.getLayout("Constants", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withPosition(12, 0);

    driveConstants.addNumber("kWheelDiameterMeters", ()->DriveConstants.kWheelDiameterMeters);
    driveConstants.addNumber("kWheelBaseLengthMeters", ()->DriveConstants.kWheelBaseLengthMeters);
    driveConstants.addNumber("kTrackWidthMeters", ()->DriveConstants.kTrackWidthMeters);
    driveConstants.addNumber("kDriveGearReduction", ()->DriveConstants.kDriveGearReduction);
    driveConstants.addNumber("kMaxDriveVelocityMetersPerSecond", ()->DriveConstants.kMaxDriveVelocityMetersPerSecond);
    driveConstants.addNumber("kMaxAngularVelocityRadiansPerSecond", ()->DriveConstants.kMaxAngularVelocityRadiansPerSecond);

    var odoemtryTab = Shuffleboard.getTab("Odometry");
    odoemtryTab.add("Field", m_field)
      .withSize(8, 5)
      .withPosition(0,0)
      .withWidget(BuiltInWidgets.kField);

    var odometryList = odoemtryTab.getLayout("Odometry", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(8,0);

    odometryList.addNumber("X (inches)", () -> Units.metersToInches(m_poseEstimator.getEstimatedPosition().getX()));
    odometryList.addNumber("Y (inches)", () -> Units.metersToInches(m_poseEstimator.getEstimatedPosition().getY()));
    odometryList.addNumber("Theta (degrees)", () -> m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());


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
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
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
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState(),
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    // Note the order of modules needs to match the order provided to DriveConstants.kDriveKinematics
    return new SwerveModulePosition[]{
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition(),
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
    m_poseEstimator.update(
        getHeading(),
        getModulePositions()
    );

    // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
    // a real robot, this must be calculated based either on latency or timestamps.

    // TODO would be cool to if this was passed in via a lambda
    // m_poseEstimator.addVisionMeasurement(
    //     ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
    //         m_poseEstimator.getEstimatedPosition()),
    //     Timer.getFPGATimestamp() - 0.3);

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
