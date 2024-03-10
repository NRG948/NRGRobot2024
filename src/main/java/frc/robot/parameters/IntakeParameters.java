/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.parameters;

import static frc.robot.parameters.MotorParameters.NeoV1_1;

/** Add your docs here. */
public enum IntakeParameters {
  PracticeBase2024(3 * 26 / 24, NeoV1_1),
  CompetitionBase2024(26 / 15, NeoV1_1);

  private final double gearRatio;
  private final MotorParameters motorParameters;

  private IntakeParameters(double gearRatio, MotorParameters motorParameters) {
    this.gearRatio = gearRatio;
    this.motorParameters = motorParameters;
  }

  public double getGearRatio() {
    return gearRatio;
  }

  public MotorParameters getMotorParameters() {
    return motorParameters;
  }
}
