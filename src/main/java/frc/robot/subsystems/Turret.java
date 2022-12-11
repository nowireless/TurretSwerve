// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.*;

public class Turret extends SubsystemBase {

  public static class DataPoint {
    public double voltage;
    public double rotations;
  }

  public enum Fault {
    kPotentiometerDisconnected,
    kPotentiometerOutOfRange
  }

  private final CANSparkMax m_motor;
  private final RelativeEncoder m_motorEncoder;
  private final SparkMaxAnalogSensor m_potentiometer;

  private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_potVoltageRotationMap = new InterpolatingTreeMap<>();

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
    // Potentiometer
    //
    m_potentiometer = m_motor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    m_potentiometer.setInverted(motorInverted);


    // Assumptions
    // - turret has 140 tooth gear
    // - potentiometer has a 16 tooth gear
    // - potentiometer is a 10 turn pot
    // - max voltage is 3.3v volts

    // //           10 Rotations   16 Pot Gear       360 degrees             10*16*360 Degrees
    // // Voltage * ------------ * --------------- * ----------- = Voltage * -----------------
    // //           3.3 Volts      140 Turret gear   1 Rotation              3.3*140   Volts
    // double potentiometerConversionFactor = (10.0/3.3) * (16.0/140.0) * 360.0;
    // m_potentiometer.setPositionConversionFactor(potentiometerConversionFactor);
    // m_potentiometer.setVelocityConversionFactor(potentiometerConversionFactor);
    m_potentiometer.setPositionConversionFactor(1); // Volts
    m_potentiometer.setVelocityConversionFactor(1); // Volts/sec

    //
    // Build potentiometer lookup table
    //

    // TODO write a unit test to verify that this can be read, and put into a interpolating tree map
    ObjectMapper objectMapper = new ObjectMapper();
    DataPoint[] dataPoints;
    try {
      File file = Paths.get(Filesystem.getDeployDirectory() + "/turret-pot-voltage-rotations.json").toFile();
      dataPoints = objectMapper.readValue(file, DataPoint[].class);
    } catch (IOException e) {
      DriverStation.reportError("Failed to read turret-pot-voltage-rotations.json", true);
      throw new RuntimeException(e);
    }

    for (var dataPoint : dataPoints) {
      System.out.println("Voltage: " + dataPoint.voltage+ ", Rotations: " + dataPoint.rotations);
      m_potVoltageRotationMap.put(
        new InterpolatingDouble(dataPoint.voltage),
        new InterpolatingDouble(dataPoint.rotations)
      );
    }

  }

  public List<Fault> getFaults() {
    List<Fault> faults = new ArrayList<>();

    if (m_potentiometer.getVoltage() < 0.1) {
      faults.add(Fault.kPotentiometerDisconnected);
    }

    if (getPotentiometerRotations() == null) {
      faults.add(Fault.kPotentiometerOutOfRange);
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
    return getPotentiometerAngle();
  }

  /**
   *
   * @return rotations or null if the potentimaters voltage is out of range;
   */
  public InterpolatingDouble getPotentiometerRotations() {
    return m_potVoltageRotationMap.getInterpolated(
        new InterpolatingDouble(m_potentiometer.getVoltage())
    );
  }

  public Rotation2d getPotentiometerAngle() {
     //             16 Pot Gear       360 degrees               16*360 Degrees
     // Rotations * --------------- * ----------- = Rotations * -----------------
     //             140 Turret gear   1 Rotation                140   Rotations


    InterpolatingDouble rotations = getPotentiometerRotations();
    if (rotations == null) {
      return new Rotation2d(); // IDK ideally if this happens the turret will have disabled itself...
    }


    return Rotation2d.fromDegrees(rotations.value * (16.0/140.0) * 360.0).plus(TurretConstants.kOffset);
  }

  public double getPotentiometerVoltage() {
    return m_potentiometer.getVoltage();
  }

  public Rotation2d getMotorAngle() {
    return Rotation2d.fromDegrees(m_motorEncoder.getPosition());
  }

  public void syncMotorEncoder() {
    m_motorEncoder.setPosition(getPotentiometerAngle().getDegrees());
  }

  public void setPower(double percent) {
    m_motor.set(percent);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Angle", getAngle().getDegrees());
    SmartDashboard.putNumber("Turret Potentiometer Angle", getPotentiometerAngle().getDegrees());
    SmartDashboard.putNumber("Turret Potentiometer Voltage", m_potentiometer.getVoltage());
    SmartDashboard.putNumber("Turret Motor Angle", getMotorAngle().getDegrees());
    SmartDashboard.putStringArray("Turret Faults", getFaultStrings());

    SmartDashboard.putNumber("Turret Angle lag", getPotentiometerAngle().minus(getMotorAngle()).getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
