/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.motors;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** An adapter class implementing a common interface on the CANSparkMax motor controller. */
public class CANSparkMaxAdapter implements MotorandEncoderAdapter {
  private CANSparkMax motor;
  private RelativeEncoder encoder;

  /**
   * Contructs an instance of this class.
   *
   * @param deviceID The CAN ID of the motor.
   * @param motorType The type of the motor.
   */
  public CANSparkMaxAdapter(int deviceID, CANSparkLowLevel.MotorType motorType) {
    motor = new CANSparkMax(deviceID, motorType);
    encoder = motor.getEncoder();
  }

  @Override
  public void set(double speed) {
    motor.set(speed);
  }

  @Override
  public double get() {
    return motor.get();
  }

  @Override
  public void setInverted(boolean isInverted) {
    motor.setInverted(isInverted);
  }

  @Override
  public boolean getInverted() {
    return motor.getInverted();
  }

  @Override
  public void disable() {
    motor.disable();
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    motor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setVelocityConversionFactor(double factor) {
    encoder.setVelocityConversionFactor(factor);
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }
}
