// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.parameters;

/** A enum representing the properties on a specific motor type. */
public enum MotorParameters {
  /**
   * A VEX PRO <a href="https://www.vexrobotics.com/217-6515.html">Falcon 500</a>
   * motor with integrated Talon FX motor controller and encoders.
   */
  Falcon500(6380.0, 4.69, 2048),

  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1650/">
   * NEO Brushless Motor V1.1</a> with integrated encoder.
   */
  NeoV1_1(5676.0, 3.75, 42),

  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1651/">
   * NEO 550 Brushless Motor</a> with integrated encoder.
   */
  Neo550(11000.0, 0.97, 42);

  private double freeSpeedRPM;
  private double stallTorque;
  private int pulsesPerRevolution;

  /**
   * Constructs an instance of this class.,
   * 
   * @param freeSpeedRPM The free speed RPM.
   * @param stallTorque  The stall torque in Nm.
   */
  MotorParameters(double freeSpeedRPM, double stallTorque, int pulsesPerRevolution) {
    this.freeSpeedRPM = freeSpeedRPM;
    this.stallTorque = stallTorque;
    this.pulsesPerRevolution = pulsesPerRevolution;
  }

  /**
   * Returns the free speed RPM of the motor.
   * 
   * @return The free speed RPM.
   */
  public double getFreeSpeedRPM() {
    return this.freeSpeedRPM;
  }

  /**
   * Returns the stall torque of the motor in Nm.
   * 
   * @return The stall torque in Nm.
   */
  public double getStallTorque() {
    return this.stallTorque;
  }

  /**
   * Returns the number of pulses per revolution of the integrated encoder.
   * 
   * @return The number of pulses per revolution of the integrated encoder.
   */
  public int getPulsesPerRevolution() {
    return this.pulsesPerRevolution;
  }
}
