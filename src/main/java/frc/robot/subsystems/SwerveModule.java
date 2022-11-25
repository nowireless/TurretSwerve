package frc.robot.subsystems;

import com.kennedyrobotics.hardware.motors.rev.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;
    private final Rotation2d steerOffset;


    public SwerveModule(SwerveModuleConfiguration config) {
        //
        // Drive motor configuration
        //
        this.driveMotor = new CANSparkMax(config.driveMotorID, CANSparkMax.MotorType.kBrushless);
        // TODO

        //
        // Steer motor configuration
        //
        this.steerMotor = new CANSparkMax(config.steerMotorID, CANSparkMax.MotorType.kBrushless);
        this.steerOffset = config.steerOffset;
        // TODO
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // TODO
        // return new SwerveModuleState(
        //   m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
        return new SwerveModuleState();
    }

//    /**
//     * Returns the current position of the module.
//     *
//     * @return The current position of the module.
//     */
//    public SwerveModulePosition getPosition() {
//        return new SwerveModulePosition(
//            m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
//    }


    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {

    }
}
