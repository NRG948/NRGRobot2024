/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.motors;

/** Add your docs here. */
public interface RelativeEncoderAdapter {
  public void setVelocityConversionFactor(double factor);

  public double getVelocity();
}
