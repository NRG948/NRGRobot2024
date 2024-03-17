/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class CANTalonFXAdapter implements MotorandEncoderAdapter {
  private TalonFX motor;

  public CANTalonFXAdapter(int deviceID) {
    motor = new TalonFX(deviceID);
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
    motor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setVelocityConversionFactor(double factor) {
    // Kraken getVelocity already returns revolutions per second
  }

  @Override
  public double getVelocity() {
    double rps = motor.getVelocity().refresh().getValueAsDouble();
    return 60 * rps;
  }
}
