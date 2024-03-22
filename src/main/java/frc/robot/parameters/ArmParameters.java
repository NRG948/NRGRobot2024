/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.parameters;

import static frc.robot.parameters.MotorParameters.NeoV1_1;

/** Add your docs here. */
public enum ArmParameters {
  PracticeBase2024(5 * 4 * 3 * 42 / 15.0, NeoV1_1, 12.5, 0.57, -11.0, 7.39, 8.0),
  CompetitionBase2024(5 * 4 * 3 * 42 / 15.0, NeoV1_1, 14.9, 0.42, -20.0, -120.75, 11.0);

  private final double gearRatio;
  private final MotorParameters motorParameters;
  private final double mass;
  public final double armLength;
  private final double stowedAngle;
  private final double rawAngleOffset;
  private final double CGAngleOffset;

  private ArmParameters(
      double gearRatio,
      MotorParameters motorParameters,
      double mass,
      double armLength,
      double stowedAngle,
      double rawAngleOffset,
      double CGAngleOffset) {
    this.gearRatio = gearRatio;
    this.motorParameters = motorParameters;
    this.mass = mass;
    this.armLength = armLength;
    this.stowedAngle = stowedAngle;
    this.rawAngleOffset = rawAngleOffset;
    this.CGAngleOffset = CGAngleOffset;
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
    return armLength;
  }

  public double getStowedAngle() {
    return stowedAngle;
  }

  public double getRawAngleOffset() {
    return rawAngleOffset;
  }

  public double getCGAngleOffset() {
    return CGAngleOffset;
  }
}
