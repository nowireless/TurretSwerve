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
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final CANSparkMax steerMotor;
    private final SparkMaxAnalogSensor steerEncoder;

    // Config
    private final Rotation2d steerOffset;

    // State
    private SwerveModuleState desiredModuleState = new SwerveModuleState();

    // NOTE: This PID controller works in Radians!
    private final PIDController steerPIDController = new PIDController(
        ModuleConstants.kPModuleTurningController,
        0,
        0
    );

    public SwerveModule(ShuffleboardLayout shuffleBoard, SwerveModuleConfiguration config) {
        //
        // Drive motor configuration
        //
        driveMotor = new CANSparkMax(config.driveMotorID, CANSparkMax.MotorType.kBrushless);
        driveMotor.setInverted(true);

        // Enable voltage compensation to 12 volts
        driveMotor.enableVoltageCompensation(ModuleConstants.kDriveVoltageCompensation);

        // Set current limit in amps
        driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveCurrentLimit);

        // Set brake mode
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Configure ramp rate
        driveMotor.setOpenLoopRampRate(0.1); // This prevents stuttering


        // Configure encoder
        driveEncoder = driveMotor.getEncoder();
        //
        // 1) Motor Rotations * kDriveGearReduction => Wheel Rotations
        // 2) PI * kWheelDiameterMeters => WheelDiameterCircumference
        // 3) Wheel rotations * WheelDiameterCircumference => Distance traveled
        //
        // With 1, 2, 3 =>  (Motor Rotations * kDriveGearReduction) * (PI * kWheelDiameterMeters ) -> Distance traveled (meters)
        double positionConversionFactor = Math.PI * ModuleConstants.kWheelDiameterMeters * ModuleConstants.kDriveGearReduction;
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

        // Configure ramp rate
        steerMotor.setOpenLoopRampRate(0.05);

        // CAN status frames
        // Status 0: Applied output/Faults/Sticky Faults/Is Follower
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        // Status 1: Motor Velocity/Motor Temperature/Motor Voltage/Motor Current
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        // Status 2: Motor Position
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        // Status 3: Analog Sensor Voltage/Analog Sensor Velocity/Analog Sensor Position
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 10); // TODO TUNE, needed
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

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Shuffleboard - Drive
        shuffleBoard.addNumber("Current Velocity", this::getVelocity);

        // Shuffleboard - Steer
        shuffleBoard.addNumber("Current Angle", () -> getAngle().getDegrees());
        shuffleBoard.addNumber("Target Angle", () -> desiredModuleState.angle.getDegrees());
        shuffleBoard.addNumber("Raw Angle", () -> getRawAngle().getDegrees());
        shuffleBoard.add("Steer PID", steerPIDController)
                .withWidget(BuiltInWidgets.kPIDController);
        shuffleBoard.addNumber("Steer PIDError", () -> Units.radiansToDegrees(steerPIDController.getPositionError()));
        shuffleBoard.addNumber("Steer output percent", steerMotor::getAppliedOutput);
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
        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredModuleState = SwerveModuleState.optimize(unoptimizedDesiredState, getAngle());

        //
        // Drive Motor
        //
        driveMotor.setVoltage(
            // The voltage is being scaled by 12, instead of kDriveVoltageCompensation.
            // If scaled by kDriveVoltageCompensation it will not be scalled corrected, as 10 volts is not the
            // modules full free speed.
            (desiredModuleState.speedMetersPerSecond / ModuleConstants.kMaxDriveVelocityMetersPerSecond) * 12.0
        );

        //
        // Steer Motor
        //

        // Calculate the steer motor output voltage from the steering PID controller.
        double steerVoltage = steerPIDController.calculate(getAngle().getRadians(), desiredModuleState.angle.getRadians());

        // Clamp the turn output output to between -10 and 10
        steerVoltage = MathUtil.clamp(steerVoltage, -ModuleConstants.kSteerVoltageCompensation, ModuleConstants.kSteerVoltageCompensation);

        // Set voltage
        steerMotor.setVoltage(steerVoltage);
    }

}
