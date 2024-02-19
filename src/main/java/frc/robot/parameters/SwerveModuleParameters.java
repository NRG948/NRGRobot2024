/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.parameters;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * A enum representing the properties on a specific swerve drive module.
 *
 * <p>The parameters for the MK4 modules are taken from <a
 * href="https://www.swervedrivespecialties.com/products/mk4-swerve-module">MK4 Swerve Module</a>.
 *
 * <p>The calculations for the theoretical maximum speeds and acceleration are taken from the <a
 * href=
 * "https://www.chiefdelphi.com/uploads/default/original/3X/f/7/f79d24101e6f1487e76099774e4ba60683e86cda.pdf">
 * FRC Drivetrain Characterization</a> paper by Noah Gleason and Eli Barnett of FRC Team 449 - The
 * Blair Robot Project.
 */
public enum SwerveModuleParameters {

  /** An MK4 Swerve Module in the L1 - Standard configuration. */
  MK4Standard(
      Units.inchesToMeters(Constants.RobotConstants.WHEEL_DIAMETER_INCHES), 8.14, 12.8, false),

  /** An MK4 Swerve Module in the L2 - Fast configuration. */
  MK4Fast(Units.inchesToMeters(Constants.RobotConstants.WHEEL_DIAMETER_INCHES), 6.75, 12.8, false),

  /** An MK4 Swerve Module in the L3 - Very Fast configuration. */
  MK4VeryFast(
      Units.inchesToMeters(Constants.RobotConstants.WHEEL_DIAMETER_INCHES), 6.12, 12.8, false),

  /** An MK4 Swerve Module in the L4 - Too Fast configuration. */
  MK4TooFast(
      Units.inchesToMeters(Constants.RobotConstants.WHEEL_DIAMETER_INCHES), 5.14, 12.8, false),

  /** An MK4I Swerve Module in the L1 - Standard configuration. */
  MK4IStandard(
      Units.inchesToMeters(Constants.RobotConstants.WHEEL_DIAMETER_INCHES),
      8.14,
      150.0 / 7.0,
      true),

  /** An MK4I Swerve Module in the L2 - Fast configuration. */
  MK4IFast(
      Units.inchesToMeters(Constants.RobotConstants.WHEEL_DIAMETER_INCHES),
      6.75,
      150.0 / 7.0,
      true),

  /** An MK4I Swerve Module in the L3 - Very Fast configuration. */
  MK4IVeryFast(
      Units.inchesToMeters(Constants.RobotConstants.WHEEL_DIAMETER_INCHES),
      6.12,
      150.0 / 7.0,
      true);

  /** */
  private final double wheelDiameter;

  private final double driveGearRatio;
  private final double steeringGearRatio;
  private final boolean steeringInverted;

  /**
   * Constructs an instance of this enum.
   *
   * @param wheelDiameter The wheel diameter in meters.
   * @param driveGearRatio The drive gear ratio.
   * @param steeringGearRatio The steering gear ratio.
   */
  SwerveModuleParameters(
      double wheelDiameter,
      double driveGearRatio,
      double steeringGearRatio,
      boolean steeringInverted) {
    this.wheelDiameter = wheelDiameter;
    this.driveGearRatio = driveGearRatio;
    this.steeringGearRatio = steeringGearRatio;
    this.steeringInverted = steeringInverted;
  }

  /**
   * Returns the wheel radius in meters.
   *
   * @return The wheel radius in meters.
   */
  public double getWheelDiameter() {
    return this.wheelDiameter;
  }

  /**
   * Returns the drive gear ratio.
   *
   * @return The drive gear ratio.
   */
  public double getDriveGearRation() {
    return this.driveGearRatio;
  }

  /**
   * Returns the steering gear ratio.
   *
   * @return The steering gear ratio.
   */
  public double getSteeringGearRatio() {
    return this.steeringGearRatio;
  }

  /**
   * Returns true if the steering motor is inverted.
   *
   * @return Whether the steering motor is inverted.
   */
  public boolean isSteeringInverted() {
    return this.steeringInverted;
  }

  /**
   * Returns the pulses per meter of the integrated encoder on the specified motor.
   *
   * @param motor
   * @return The pulses per meter of the integrated encoder on the specified motor.
   */
  public double calculateDrivePulsesPerMeter(MotorParameters motor) {
    return (motor.getPulsesPerRevolution() * this.driveGearRatio) / (this.wheelDiameter * Math.PI);
  }

  /**
   * Returns the theoretical maximum drive speed in m/s when using the specified motor.
   *
   * @param motor The motor parameters.
   * @return The theoretical maximum drive speed.
   */
  public double calculateMaxDriveSpeed(MotorParameters motor) {
    return (motor.getFreeSpeedRPM() * this.wheelDiameter * Math.PI) / (60.0 * this.driveGearRatio);
  }

  /**
   * Returns the theoretical maximum drive acceleration in m/s^2 when using the specified motor.
   *
   * @param motor The motor parameters.
   * @param robotMass the total robot mass in Kg including bumpers and battery.
   * @return The theoretical maximum drive acceleration.
   */
  public double calculateMaxDriveAcceleration(MotorParameters motor, double robotMass) {
    return (2 * 4 * motor.getStallTorque() * this.driveGearRatio)
        / (this.wheelDiameter * robotMass);
  }

  /**
   * Returns the theoretical maximum steering speed in rad/s when using the specified motor.
   *
   * @param motor The motor parameters.
   * @return The theoretical maximum drive speed.
   */
  public double calculateMaxSteeringSpeed(MotorParameters motor) {
    return (motor.getFreeSpeedRPM() * 2 * Math.PI) / (60.0 * this.steeringGearRatio);
  }

  /**
   * Returns the theoretical maximum drive acceleration in rad/s^2 when using the specified motor.
   *
   * @param motor The motor parameters.
   * @param robotMass the total robot mass in Kg including bumpers and battery.
   * @return The theoretical maximum drive acceleration.
   */
  public double calculateMaxSteeringAcceleration(MotorParameters motor, double robotMass) {
    return (2 * 4 * motor.getStallTorque() * this.steeringGearRatio * 2 * Math.PI) / robotMass;
  }
}
