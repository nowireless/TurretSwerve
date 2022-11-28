package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.kennedyrobotics.hardware.motors.ctre.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import static frc.robot.Constants.*;

public class Elevator extends SubsystemBase {
    private static final String NAME = "elevator";

    public enum ControlState {
        Neutral,
        OpenLoop,
        PositionPID,
    }

    private class PeriodicIO {
        // INPUT
        double elevatorExtensionRaw;
        double elevatorExtensionInches;
        boolean isAtLowLimit;
        double feedForward; // Can be used to offset the cube when it is present in the elevator

        double outputPercent;
        double leaderMotorCurrent;
        double followerMotorCurrent;

        // OUTPUT
        double demand;
    }


    // Hardware
    private final TalonFX liftMotorLeader;
    private final TalonFX liftMotorFollower;
    private final Solenoid liftBrake;

    // State
    ControlState controlState = ControlState.OpenLoop;
    PeriodicIO periodicIO = new PeriodicIO();
    boolean hasBeenZeroed;


    // Dependencies
    private final Supplier<Boolean> intakeHasCubeSupplier;

    public Elevator(Supplier<Boolean> intakeHasCubeSupplier) {
        setName(NAME);
        this.intakeHasCubeSupplier = intakeHasCubeSupplier;

        // Hardware
        // There are 2 lift motors, one will be the leader, and other other will be the follower.
        // The leader will be configured as follows:
        // - Use the internal encoder to determine the height of the elevator
        // - A normally open limit switch will be used to determine when the elevator is at the flower limit
        // - When the elevator is at the lower limit the encoder postion will be reset.
        // - Positive Encoder values mean up
        // - Positive Motor command values mean up
        //
        liftMotorLeader = new TalonFX(ElevatorConstants.kMotorLeaderID);
        liftMotorLeader.setInverted(true);
        liftMotorLeader.setNeutralMode(NeutralMode.Brake);
        liftMotorLeader.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 100);
        liftMotorLeader.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor, 0, 100);
        liftMotorLeader.configClearPositionOnLimitR(true, 100);

        // TODO CAN status periods

        // Follower configuration
        liftMotorFollower = new TalonFX(ElevatorConstants.kMotorFollowerID);
        liftMotorFollower.follow(liftMotorLeader);
        liftMotorFollower.setInverted(TalonFXInvertType.FollowMaster);
        liftMotorFollower.setNeutralMode(NeutralMode.Brake);

        // TODO CAN status periods

        // The lift break is a single solenoid, that by default does not break the elevator. It has to be commanded to
        // break.
        liftBrake = new Solenoid(PneumaticsModuleType.CTREPCM, ElevatorConstants.kSolenoidLiftBrakeChannel);
    }

    //
    // Control loop and Periodic IO
    //

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            // When the robot is disabled lets zero the elevator when its at the low limit
            resetIfAtLimit();
        }

        // Read inputs
        readPeriodicInputs();

        // Write output
        writePeriodicOutputs();

        outputTelemetry();
    }

    public void readPeriodicInputs() {
        periodicIO.isAtLowLimit = liftMotorLeader.isRevLimitSwitchClosed() == 1;


        // Encoder counts to elevator extension in inches. TODO Need to add the scale value to the height calculate
        // 1 Revolution   ? Inches
        // ------------ * ------------
        // 2048 CPR       1 Revolution
        periodicIO.elevatorExtensionRaw = liftMotorLeader.getSelectedSensorPosition(0);
        periodicIO.elevatorExtensionInches = periodicIO.elevatorExtensionRaw * 1.0;

        // Query the state of the take subsystem to determine if the intake has a cube, if so apply a feedforward value
        // to compensate for the weight of the cube.
        if (intakeHasCubeSupplier.get()) {
            periodicIO.feedForward = 0.0; // TODO
        } else {
            periodicIO.feedForward = 0.0;
        }


        // Motor stats
        periodicIO.outputPercent = liftMotorLeader.getMotorOutputPercent();

        periodicIO.leaderMotorCurrent = liftMotorLeader.getStatorCurrent();     // CTREMotorUtil.getStatorCurrent(liftMotorLeader);
        periodicIO.followerMotorCurrent = liftMotorFollower.getStatorCurrent(); // CTREMotorUtil.getStatorCurrent(liftMotorFollower);
    }

    public void writePeriodicOutputs() {
        if (controlState == ControlState.OpenLoop) {
            System.out.println("[Elevator] OpenLoop");

            liftMotorLeader.set(ControlMode.PercentOutput, periodicIO.demand, DemandType.ArbitraryFeedForward, periodicIO.feedForward);

            if (periodicIO.demand == 0.0) {
                // When the elevator is not being command to move set the break
                if (!liftBrake.get() && DriverStation.isEnabled()) {
                    System.out.println("Turning brake on");
                }
                liftBrake.set(true);
            } else {
                // WHen the elevator is being commanded to move release the break
                if (liftBrake.get() && DriverStation.isEnabled()) {
                    System.out.println("Turning brake off");
                }
                liftBrake.set(false);
            }

        } else if (controlState == ControlState.PositionPID) {
            System.out.println("[Elevator] PositionPID");

            // TODO Need to add PID of some sort
            System.out.println("PositionPID does not work yet");
            liftMotorLeader.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
        } else if (controlState == ControlState.Neutral){
            System.out.println("[Elevator] In Neutral!" );
            // Neutral Output doesn't stop the follower talon
            // liftMotorLeader.neutralOutput();
            // TODO Report a bug, where neutral mode doesn't stop a motor, when a Set with a PercentOutput with ArbitrayFeedForward.
            // As a WAR use the following to stop both motors
            liftMotorLeader.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
            liftBrake.set(true);
        } else {
            System.out.println("[Elevator] Unknown State: " + controlState);
            DriverStation.reportError("Elevator in unknown state: "+controlState.toString(), false);
            liftMotorLeader.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
            liftBrake.set(false);
        }
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Elevator Control State", controlState.toString());
        SmartDashboard.putBoolean("Elevator has been zeroed", hasBeenZeroed());
        SmartDashboard.putNumber("Elevator Output %", periodicIO.outputPercent);
        SmartDashboard.putNumber("Elevator Current (Leader)", periodicIO.leaderMotorCurrent);
        SmartDashboard.putNumber("Elevator Current (Follower)", periodicIO.followerMotorCurrent);
        SmartDashboard.putNumber("Elevator Height", getInchesOffGround());
        SmartDashboard.putNumber("Elevator Height Raw", periodicIO.elevatorExtensionRaw);
        SmartDashboard.putBoolean("Elevator Low Limit", periodicIO.isAtLowLimit);
        SmartDashboard.putNumber("Elevator Demand", periodicIO.demand);
        SmartDashboard.putNumber("Elevator FeedForward", periodicIO.feedForward);
    }

    //
    // Getters for Elevator State
    //

    /**
     * Height of intake from the ground
     * @return height in inches
     */
    public double getInchesOffGround() {
        return periodicIO.elevatorExtensionInches + ElevatorConstants.kIntakeHeightOffset;
    }

    /**
     * Height of how far the elevator has extended from its home position
     * @return extension in inches.
     */
    public double getExtension() {
        return periodicIO.elevatorExtensionInches;
    }

    /**
     * Has the elevator reach the low limit
     * @return if true then the elevator is at the low limit
     */
    public boolean isAtLowLimit() {
        return periodicIO.isAtLowLimit;
    }


    /**
     * Set point in inches for closed loop contol states (PositionPID)
     * @return if in closed loop control mode the height in inches, otherwise NaN
     */
    public double getSetPoint() {
        if (controlState == ControlState.PositionPID) {
            return periodicIO.demand;
        }

        return Double.NaN;
    }

    public boolean hasBeenZeroed() {
        return hasBeenZeroed;
    }


    //
    // Sensor helpers
    //

    /**
     * Reset Sensors if the elevator is at the low limit
     */
    public void resetIfAtLimit() {
        if (this.isAtLowLimit()) {
            zeroSensors();
        }
    }


    /**
     * Current control state of the elevator
     * @return the state
     */
    public ControlState getControlState() {
        return controlState;
    }

    //
    // Modify/Control Elevator State
    //
    public void zeroSensors() {
        liftMotorLeader.setSelectedSensorPosition(0, 0, 20);
        hasBeenZeroed = true;
    }

    public void setOpenLoop(double percentage) {
        controlState = ControlState.OpenLoop;
        periodicIO.demand = percentage;
    }

    public void neutral() {
        controlState = ControlState.Neutral;
        periodicIO.demand = 0;
    }
}
