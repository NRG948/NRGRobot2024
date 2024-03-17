/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.motors;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** An interface that abstracts motor controllers from various libraries. */
public interface MotorandEncoderAdapter extends MotorController {
  /**
   * Sets brake or coast mode on the motor controller.
   *
   * @param enabled If true, sets the motor controller to brake mode; otherwise sets it to coast
   *     mode.
   */
  public void setBrakeMode(boolean enabled);

  public void setVelocityConversionFactor(double factor);

  public double getVelocity();
}
