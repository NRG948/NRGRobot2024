// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Represents swerve module velocities. */
public class SwerveModuleVelocities {
  /** The translational velocity of the wheel in m/s. */
  public final double driveVelocity;

  /** The rotational velocity of the wheel in rad/s */
  public final double steeringVelocity;

  /**
   * Creates a new SwerveModuleVelocities.
   * 
   * @param driveVelocity    The translational velocity of the wheel in m/s.
   * @param steeringVelocity The rotational velocity of the wheel in rad/s.
   */
  public SwerveModuleVelocities(double driveVelocity, double steeringVelocity) {
    this.driveVelocity = driveVelocity;
    this.steeringVelocity = steeringVelocity;
  }
}
