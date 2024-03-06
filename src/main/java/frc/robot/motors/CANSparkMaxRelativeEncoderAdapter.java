/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.motors;

import com.revrobotics.RelativeEncoder;

/** An adapter class implementing a common interface on the CANSparkMax relative encoder. */
public class CANSparkMaxRelativeEncoderAdapter implements RelativeEncoderAdapter {
  private RelativeEncoder encoder;

  /**
   * Contructs an instance of this class.
   *
   * @param encoder The relative encoder.
   */
  CANSparkMaxRelativeEncoderAdapter(RelativeEncoder encoder) {
    this.encoder = encoder;
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
