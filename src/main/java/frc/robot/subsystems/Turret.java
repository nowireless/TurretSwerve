// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.team254.lib.util.InterpolatingDouble;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.*;

public class Turret extends SubsystemBase {

  private static final double kEncoderResetIterations = 500;
  private static final double kEncoderResetMaxAngularVelocity = 0.5; // Degrees per second


  public static class DataPoint {
    public double voltage;
    public double motorAngle;
  }

  public enum Fault {
    kAbsEncoderHardwareFault,
    kAbsEncoderAPIError,
    kAbsEncoderUnderVoltage,
    kAbsEncoderResetDuringEnabled,
    kAbsEncoderMagnetTooWeak,
    kMotorEncoderNotSynced;
  }

  private final CANSparkMax m_motor;
  private final RelativeEncoder m_motorEncoder;
  private final CANCoder m_absoluteEncoder;
  private int m_encoderResetIteration;
  private boolean m_motorEncoderSynced;

  /** Creates a new ExampleSubsystem. */
  public Turret() {
    //
    // Motor
    //
    m_motor = new CANSparkMax(TurretConstants.kMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

    final boolean motorInverted = false;
    m_motor.setInverted(motorInverted);
    m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_motor.enableVoltageCompensation(10);
    m_motor.setSmartCurrentLimit(40);
    m_motor.setOpenLoopRampRate(0.1);
    m_motor.setClosedLoopRampRate(0.1);

    // CAN status frames
    // Status 0: Applied output/Faults/Sticky Faults/Is Follower
    m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    // Status 1: Motor Velocity/Motor Temperature/Motor Voltage/Motor Current
    m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
    // Status 2: Motor Position
    m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    // Status 3: Analog Sensor Voltage/Analog Sensor Velocity/Analog Sensor Position
    m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 20);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 170);
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -170);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    //
    // Motor Encoder
    //
    m_motorEncoder = m_motor.getEncoder();

    //             10 Motor Gear     360 degrees                   10 * 360 Degrees
    // Rotations * --------------- * --------------- = Rotations * -----------------
    //             140 Turret gear   1 Rotation                    140 Rotations

    //                  1 Motor Rotation   1 output shaft turn   10 Motor Gear     360 Degrees
    // Encoder Counts * ---------------- * ------------------- * --------------- * -----------
    //                  42 Counts          45 Motor Rotation     140 Turret Gear   1 Turret Rotation
    double motorEncoderConversionFactor = (1.0/45.0) * (10.0/140.0) * 360.0;
    m_motorEncoder.setPositionConversionFactor(motorEncoderConversionFactor);
    m_motorEncoder.setVelocityConversionFactor(motorEncoderConversionFactor);


    //
    // Absolute Encoder
    //
    m_absoluteEncoder = new CANCoder(TurretConstants.kEncoderID);
    CANCoderConfiguration absoluteEncoderConfiguration = new CANCoderConfiguration();
    absoluteEncoderConfiguration.sensorDirection = true;
    absoluteEncoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    absoluteEncoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    absoluteEncoderConfiguration.magnetOffsetDegrees = TurretConstants.kOffset.getDegrees();
    m_absoluteEncoder.configAllSettings(absoluteEncoderConfiguration);
  }

  public List<Fault> getFaults() {
    List<Fault> faults = new ArrayList<>();

    CANCoderFaults absoluteEncoderFaults = new CANCoderFaults();
    m_absoluteEncoder.getFaults(absoluteEncoderFaults);


    if (absoluteEncoderFaults.HardwareFault) {
      faults.add(Fault.kAbsEncoderHardwareFault);
    }

    if (absoluteEncoderFaults.APIError) {
      faults.add(Fault.kAbsEncoderAPIError);
    }

    if (absoluteEncoderFaults.UnderVoltage) {
      faults.add(Fault.kAbsEncoderUnderVoltage);
    }

    if (absoluteEncoderFaults.ResetDuringEn) {
      faults.add(Fault.kAbsEncoderResetDuringEnabled);
    }

    if (absoluteEncoderFaults.MagnetTooWeak) {
      faults.add(Fault.kAbsEncoderMagnetTooWeak);
    }

    if (!m_motorEncoderSynced) {
      faults.add(Fault.kMotorEncoderNotSynced);
    }

    return faults;
  }

  public String[] getFaultStrings() {
    var faults = getFaults();
    String[] result = new String[faults.size()];
    for (int i = 0; i < faults.size(); i++) {
      result[i] = faults.get(i).toString();
    }
    return result;
  }

  /**
   * This is the real angle of the turret.
   * @return turret angle
   */
  public Rotation2d getAngle() {
    // For right now we will use the pot for the angle.
    // Alternatively the motor angle could be used, and be seeded from the pot angle.
    return getMotorAngle();
  }


  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromDegrees(m_absoluteEncoder.getAbsolutePosition());
  }

  public Rotation2d getMotorAngle() {
    return Rotation2d.fromDegrees(m_motorEncoder.getPosition());
  }

  public void setPower(double percent) {
    m_motor.set(percent);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //
    // Handle syncing positions between the absolute encoder and NEO encoder
    //

    // Reset the NEO's encoder periodically when the turret is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
    // end up getting a good reading. If we reset periodically this won't matter anymore.
    if (m_motorEncoder.getVelocity() <  kEncoderResetMaxAngularVelocity) {
      if (++m_encoderResetIteration >= kEncoderResetIterations) {
        m_encoderResetIteration = 0;
        m_motorEncoder.setPosition(getAbsoluteAngle().getDegrees());
        m_motorEncoderSynced = true;
      }
    } else {
      m_encoderResetIteration = 0;
    }

    //
    // Dashboard data
    //
    SmartDashboard.putNumber("Turret Angle", getAngle().getDegrees());
    SmartDashboard.putNumber("Turret Absolute Angle", getAbsoluteAngle().getDegrees());
    SmartDashboard.putNumber("Turret Encoder Reset Iterations", m_encoderResetIteration);
    SmartDashboard.putBoolean("Turret Motor Encoder synced", m_motorEncoderSynced);
    SmartDashboard.putStringArray("Turret Faults", getFaultStrings());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
