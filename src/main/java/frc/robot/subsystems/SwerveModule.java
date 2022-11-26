package frc.robot.subsystems;

import com.kennedyrobotics.hardware.motors.rev.CANSparkMax;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import static frc.robot.Constants.*;

public class SwerveModule {

    public static class SwerveModuleConfiguration {
        public final int driveMotorID;
        public final int steerMotorID;
        public final Rotation2d steerOffset;


        public SwerveModuleConfiguration(int driveMotorID, int steerMotorID, Rotation2d steerOffset) {
            this.driveMotorID = driveMotorID;
            this.steerMotorID = steerMotorID;
            this.steerOffset = steerOffset;
        }
    }

    // Hardware
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final CANSparkMax steerMotor;
    private final SparkMaxAnalogSensor steerEncoder;
    private final SparkMaxPIDController steerPID;

    // Config
    private final Rotation2d steerOffset;

    // State
    private SwerveModuleState desiredModuleState = new SwerveModuleState();

    public SwerveModule(ShuffleboardLayout shuffleBoard, SwerveModuleConfiguration config) {
        //
        // Drive motor configuration
        //
        driveMotor = new CANSparkMax(config.driveMotorID, CANSparkMax.MotorType.kBrushless);
        driveMotor.setInverted(false);

        // Enable voltage compensation to 12 volts
        driveMotor.enableVoltageCompensation(ModuleConstants.kDriveVoltageCompensation);

        // Set current limit in amps
        driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveCurrentLimit);

        // Set brake mode
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Configure encoder
        driveEncoder = driveMotor.getEncoder();
        double positionConversionFactor = Math.PI * ModuleConstants.kWheelDiameterMeters * ModuleConstants.kWheelDiameterMeters;
        driveEncoder.setPositionConversionFactor(positionConversionFactor);
        driveEncoder.setVelocityConversionFactor(positionConversionFactor / 60.0);

        // CAN status frames
        // Status 0: Applied output/Faults/Sticky Faults/Is Follower
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        // Status 1: Motor Velocity/Motor Temperature/Motor Voltage/Motor Current
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        // Status 2: Motor Position
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        // Status 3: Analog Sensor Voltage/Analog Sensor Velocity/Analog Sensor Position
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 50); // TODO TUNE, not needed
        // Status 4: Alternate Encoder Velocity/Alternate Encoder Position
        // TODO this does not look settable

        //
        // Steer motor configuration
        //
        final boolean invertSteerMotor = true;
        steerMotor = new CANSparkMax(config.steerMotorID, CANSparkMax.MotorType.kBrushless);
        steerMotor.setInverted(invertSteerMotor);
        steerOffset = config.steerOffset;

        // Enable voltage compensation to 12 volts
        steerMotor.enableVoltageCompensation(ModuleConstants.kSteerVoltageCompensation);

        // Set current limit in amps
        steerMotor.setSmartCurrentLimit(ModuleConstants.kSteerCurrentLimit);

        // Set brake mode
        steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // CAN status frames
        // Status 0: Applied output/Faults/Sticky Faults/Is Follower
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        // Status 1: Motor Velocity/Motor Temperature/Motor Voltage/Motor Current
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        // Status 2: Motor Position
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        // Status 3: Analog Sensor Voltage/Analog Sensor Velocity/Analog Sensor Position
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 20); // TODO TUNE, needed
        // Status 4: Alternate Encoder Velocity/Alternate Encoder Position
        // TODO this does not look settable


        // Steer Encoder
        steerEncoder = steerMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        steerEncoder.setInverted(!invertSteerMotor); // Opposite phase
        // Encoder is 1 to 1 gearing with the module.
        //           1 Revolution   360 Degrees
        // Voltage * ------------ * ------------ = Voltage * 360.0/3.3
        //           3.3 Volts      1 Revolution
        steerEncoder.setPositionConversionFactor(360.0/3.3); // Degrees

        // Volts    1 Revolution   360 Degrees    Voltage
        // ------ * ------------ * ------------ = ------- * 360.0/3.3
        // Second   3.3 Volts      1 Revolution   Second
        steerEncoder.setVelocityConversionFactor((360.0/3.3)); // Degrees per second

        // Steer PID
        steerPID = steerMotor.getPIDController();
        steerPID.setP(ModuleConstants.kSteerPIDProportional);
        steerPID.setI(ModuleConstants.kSteerPIDIntegral);
        steerPID.setD(ModuleConstants.kSteerPIDDerivative);
        steerPID.setFeedbackDevice(steerEncoder);


        // Shuffleboard
        shuffleBoard.addNumber("Current Angle", () -> getAngle().getDegrees());
        shuffleBoard.addNumber("Target Angle", () -> desiredModuleState.angle.getDegrees());
        shuffleBoard.addNumber("Current Velocity", this::getVelocity);

        shuffleBoard.addNumber("Raw Angle", () -> getRawAngle().getDegrees());

    }

    /**
     *
     * @return
     */
    public Rotation2d getAngle() {
        return getRawAngle().plus(steerOffset);
    }

    public Rotation2d getRawAngle() {
        return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }
    /**
     * Drive motor velocity
     * @return velocity in meters/second
     */
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param unoptimizedDesiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState unoptimizedDesiredState) {
        // Apply the module offset before optimizing the angle, other wise the modules will freak out if we apply the
        // offset manually when retrieving the angle, and when setting the PID controller
        unoptimizedDesiredState.angle = unoptimizedDesiredState.angle.minus(steerOffset);

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredModuleState = SwerveModuleState.optimize(unoptimizedDesiredState, getRawAngle());

        driveMotor.setVoltage(
            (desiredModuleState.speedMetersPerSecond / ModuleConstants.kMaxVelocityMetersPerSecond) * ModuleConstants.kDriveVoltageCompensation
        );

        steerPID.setReference(desiredModuleState.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    }

}
