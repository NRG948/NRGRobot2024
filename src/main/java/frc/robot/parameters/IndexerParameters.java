/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.parameters;

import static frc.robot.parameters.MotorParameters.NeoV1_1;

/** Add your docs here. */
public enum IndexerParameters {
  PracticeBase2024(3 * 26 / 15, 0.033, 0.5, NeoV1_1, true),
  CompetitionBase2024(3 * 26 / 15, 0.033, 0.5, NeoV1_1, false);

  private final double gearRatio;
  private final double diameter;
  private final double mass;
  private final MotorParameters motorParameters;
  private final boolean isInverted;

  private IndexerParameters(
      double gearRatio,
      double diameter,
      double mass,
      MotorParameters motorParameters,
      boolean isInverted) {
    this.gearRatio = gearRatio;
    this.diameter = diameter;
    this.mass = mass;
    this.motorParameters = motorParameters;
    this.isInverted = isInverted;
  }

  public double getGearRatio() {
    return gearRatio;
  }

  public double getDiameter() {
    return diameter;
  }

  public double getMass() {
    return mass;
  }

  public boolean getInverted() {
    return isInverted;
  }

  public MotorParameters getMotorParameters() {
    return motorParameters;
  }
}
