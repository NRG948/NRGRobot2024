/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.parameters;

import static frc.robot.parameters.MotorParameters.KrakenX60;
import static frc.robot.parameters.MotorParameters.NeoV1_1;

/** Add your docs here. */
public enum ArmParameters {
  PracticeBase2024(5 * 4 * 3 * 42 / 15.0, NeoV1_1, 12.5, 0.57),
  CompetitionBase2024(5 * 4 * 3 * 42 / 15.0, KrakenX60, 12.5, 0.57);

  private final double gearRatio;
  private final MotorParameters motorParameters;
  private final double mass;
  public final double arm_length;

  private ArmParameters(
      double gearRatio, MotorParameters motorParameters, double mass, double arm_length) {
    this.gearRatio = gearRatio;
    this.motorParameters = motorParameters;
    this.mass = mass;
    this.arm_length = arm_length;
  }

  public double getGearRatio() {
    return gearRatio;
  }

  public MotorParameters getMotorParameters() {
    return motorParameters;
  }

  public double getMass() {
    return mass;
  }

  public double getArmLength() {
    return arm_length;
  }
}
