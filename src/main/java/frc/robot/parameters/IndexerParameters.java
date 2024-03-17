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
    PracticeBase2024(3 * 26 / 15, 0.033, 0.5, NeoV1_1),
    CompetitionBase2024(3 * 26 / 15, 0.033, 0.5, NeoV1_1); //TODO: double check values

    private final double gearRatio;
    private final double diameter;
    private final double mass;
    private final MotorParameters motorParameters;

    private IndexerParameters(double gearRatio, double diameter, double mass, MotorParameters motorParameters) {
        this.gearRatio = gearRatio;
        this.diameter = diameter;
        this.mass = mass;
        this.motorParameters = motorParameters;
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

    public MotorParameters getMotorParameters() {
        return motorParameters;
    }
}