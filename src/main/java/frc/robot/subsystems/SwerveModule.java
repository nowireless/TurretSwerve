package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
    private final CANSparkMax m_driveMotor;
    private final RelativeEncoder m_driveEncoder;
    private final CANSparkMax m_steerMotor;
    private final SparkMaxAnalogSensor m_steerEncoder;

    // Config
    private final Rotation2d m_steerOffset;

    // State
    private SwerveModuleState m_desiredModuleState = new SwerveModuleState();

    // NOTE: This PID controller works in Radians!
    private final PIDController m_steerPIDController = new PIDController(
        ModuleConstants.kPModuleTurningController,
        0,
        0
    );

    public SwerveModule(ShuffleboardLayout shuffleBoard, SwerveModuleConfiguration config) {
        //
        // Drive motor configuration
        //
        m_driveMotor = new CANSparkMax(config.driveMotorID, CANSparkMax.MotorType.kBrushless);
        m_driveMotor.setInverted(true);

        // Enable voltage compensation to 12 volts
        m_driveMotor.enableVoltageCompensation(ModuleConstants.kDriveVoltageCompensation);

        // Set current limit in amps
        m_driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveCurrentLimit);

        // Set brake mode
        m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Configure ramp rate
        m_driveMotor.setOpenLoopRampRate(0.1); // This prevents stuttering


        // Configure encoder
        m_driveEncoder = m_driveMotor.getEncoder();
        //
        // 1) Motor Rotations * kDriveGearReduction => Wheel Rotations
        // 2) PI * kWheelDiameterMeters => WheelDiameterCircumference
        // 3) Wheel rotations * WheelDiameterCircumference => Distance traveled
        //
        // With 1, 2, 3 =>  (Motor Rotations * kDriveGearReduction) * (PI * kWheelDiameterMeters ) -> Distance traveled (meters)
        double positionConversionFactor = Math.PI * ModuleConstants.kWheelDiameterMeters * ModuleConstants.kDriveGearReduction;
        m_driveEncoder.setPositionConversionFactor(positionConversionFactor);
        m_driveEncoder.setVelocityConversionFactor(positionConversionFactor / 60.0);

        // CAN status frames
        // Status 0: Applied output/Faults/Sticky Faults/Is Follower
        m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        // Status 1: Motor Velocity/Motor Temperature/Motor Voltage/Motor Current
        m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        // Status 2: Motor Position
        m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        // Status 3: Analog Sensor Voltage/Analog Sensor Velocity/Analog Sensor Position
        m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 50); // TODO TUNE, not needed
        // Status 4: Alternate Encoder Velocity/Alternate Encoder Position
        // TODO this does not look settable

        //
        // Steer motor configuration
        //
        final boolean invertSteerMotor = true;
        m_steerMotor = new CANSparkMax(config.steerMotorID, CANSparkMax.MotorType.kBrushless);
        m_steerMotor.setInverted(invertSteerMotor);
        m_steerOffset = config.steerOffset;

        // Enable voltage compensation to 12 volts
        m_steerMotor.enableVoltageCompensation(ModuleConstants.kSteerVoltageCompensation);

        // Set current limit in amps
        m_steerMotor.setSmartCurrentLimit(ModuleConstants.kSteerCurrentLimit);

        // Set brake mode
        m_steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Configure ramp rate
        m_steerMotor.setOpenLoopRampRate(0.05);

        // CAN status frames
        // Status 0: Applied output/Faults/Sticky Faults/Is Follower
        m_steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        // Status 1: Motor Velocity/Motor Temperature/Motor Voltage/Motor Current
        m_steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        // Status 2: Motor Position
        m_steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        // Status 3: Analog Sensor Voltage/Analog Sensor Velocity/Analog Sensor Position
        m_steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 10); // TODO TUNE, needed
        // Status 4: Alternate Encoder Velocity/Alternate Encoder Position
        // TODO this does not look settable


        // Steer Encoder
        m_steerEncoder = m_steerMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        m_steerEncoder.setInverted(!invertSteerMotor); // Opposite phase
        // Encoder is 1 to 1 gearing with the module.
        //           1 Revolution   360 Degrees
        // Voltage * ------------ * ------------ = Voltage * 360.0/3.3
        //           3.3 Volts      1 Revolution
        m_steerEncoder.setPositionConversionFactor(360.0/3.3); // Degrees

        // Volts    1 Revolution   360 Degrees    Voltage
        // ------ * ------------ * ------------ = ------- * 360.0/3.3
        // Second   3.3 Volts      1 Revolution   Second
        m_steerEncoder.setVelocityConversionFactor((360.0/3.3)); // Degrees per second

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Shuffleboard - Drive
        shuffleBoard.addNumber("Current Velocity", this::getVelocity);

        // Shuffleboard - Steer
        shuffleBoard.addNumber("Current Angle", () -> getAngle().getDegrees());
        shuffleBoard.addNumber("Target Angle", () -> m_desiredModuleState.angle.getDegrees());
        shuffleBoard.addNumber("Raw Angle", () -> getRawAngle().getDegrees());
        shuffleBoard.add("Steer PID", m_steerPIDController)
                .withWidget(BuiltInWidgets.kPIDController);
        shuffleBoard.addNumber("Steer PIDError", () -> Units.radiansToDegrees(m_steerPIDController.getPositionError()));
        shuffleBoard.addNumber("Steer output percent", m_steerMotor::getAppliedOutput);
    }

    /**
     *
     * @return
     */
    public Rotation2d getAngle() {
        return getRawAngle().plus(m_steerOffset);
    }

    public Rotation2d getRawAngle() {
        return Rotation2d.fromDegrees(m_steerEncoder.getPosition());
    }
    /**
     * Drive motor velocity
     * @return velocity in meters/second
     */
    public double getVelocity() {
        return m_driveEncoder.getVelocity();
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
        // Optimize the reference state to avoid spinning further than 90 degrees
        m_desiredModuleState = SwerveModuleState.optimize(unoptimizedDesiredState, getAngle());

        //
        // Drive Motor
        //
        m_driveMotor.setVoltage(
            // The voltage is being scaled by 12, instead of kDriveVoltageCompensation.
            // If scaled by kDriveVoltageCompensation it will not be scalled corrected, as 10 volts is not the
            // modules full free speed.
            (m_desiredModuleState.speedMetersPerSecond / ModuleConstants.kMaxDriveVelocityMetersPerSecond) * 12.0
        );

        //
        // Steer Motor
        //

        // Calculate the steer motor output voltage from the steering PID controller.
        double steerVoltage = m_steerPIDController.calculate(getAngle().getRadians(), m_desiredModuleState.angle.getRadians());

        // Clamp the turn output output to between -10 and 10
        steerVoltage = MathUtil.clamp(steerVoltage, -ModuleConstants.kSteerVoltageCompensation, ModuleConstants.kSteerVoltageCompensation);

        // Set voltage
        m_steerMotor.setVoltage(steerVoltage);
    }

}
