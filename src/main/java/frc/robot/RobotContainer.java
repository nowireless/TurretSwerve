// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveWithController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.kennedyrobotics.auto.AutoSelector;
import com.kennedyrobotics.hardware.misc.RevDigit;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // OI
  private final CommandXboxController m_controller = new CommandXboxController(0);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final ShooterHood m_shooterHood = new ShooterHood();
  private final Vision m_vision = new Vision(); 
  private final Turret m_turret = new Turret();

  //Auto
  private final RevDigit m_revDigit;
  private final AutoSelector m_autoSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure default commands
    m_drivetrain.setDefaultCommand(new DriveWithController(m_drivetrain, m_controller.getHID()));

    // Configure the button bindings
    configureButtonBindings();

    // Path planner stuff
    PathPlannerServer.startServer(5811);

    // Auto selector
    m_revDigit = new RevDigit();
    m_revDigit.display("Ryan");
    m_autoSelector = new AutoSelector(m_revDigit, "DFLT", new SequentialCommandGroup(
      new PrintCommand("Hi")
    ));

    // Initialize other autos here
    m_autoSelector.registerCommand("WPILibSwerveExample", "WPIS", makeWPILibSwerveExample());
    // m_autoSelector.registerCommand("PathPlannerExample", "PPLE", makePathPlannerExample());

    m_autoSelector.initialize();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // TODO this should also do something with odometry? As it freaks out 
    m_controller.start().onTrue(new InstantCommand(m_drivetrain::zeroHeading)); 
    m_controller.back().onTrue(new InstantCommand(()->m_drivetrain.resetOdometry(new Pose2d()))); 

    m_controller.a().whileTrue(new FunctionalCommand(
      () -> {},
      () -> {
        double power = m_controller.getRightX();
        m_turret.setPower(Math.copySign(power * power, power));
      },
      (interrupted) -> m_turret.setPower(0),
      () -> false,
      m_turret
    ));

    m_controller.rightStick().toggleOnTrue(new RunCommand(()->{
      var latchedModuleStates = new SwerveModuleState[]{
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    };

    m_drivetrain.setModuleStates(latchedModuleStates);
   }, m_drivetrain));
  }

  public void onTeleopInit() {
  }

  public void onAutonomousInit() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoSelector.selected();
  }

  //
  // Autontous modes below
  //
  public Command makeWPILibSwerveExample() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
            1,
            1)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         // List.of(new Translation2d(1, 1)),
    //         List.of(),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(Units.inchesToMeters(24), 0, new Rotation2d(0)),
    //         config);
    Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(24), Units.inchesToMeters(24))),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(48), 0, new Rotation2d(0)),
        config);
    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_drivetrain::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(3, 0, 0),
        new PIDController(3, 0, 0),
        thetaController,
        m_drivetrain::setModuleStates,
        m_drivetrain
    );

    m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    return new SequentialCommandGroup(
      // Reset odometry to the starting pose of the trajectory.
      new InstantCommand(() -> m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose())),
      // Run path following command, then stop at the end.
      swerveControllerCommand.andThen(() -> m_drivetrain.drive(0, 0, 0, false))
    );
  }

  public Command makePathPlannerExample() {
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("intakeDown", new PrintCommand("Intake Down!"));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      m_drivetrain::getPose, // Pose2d supplier
      m_drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      m_drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      m_drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);
  }
}
