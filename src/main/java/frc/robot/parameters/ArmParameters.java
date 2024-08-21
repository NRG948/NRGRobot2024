// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.parameters;

/** A enum representing the properties on a specific motor type. */
public enum ArmParameters {
    ArmRedesign(6.55088578738, 1, -26, -22, 80, MotorParameters.NeoV1_1);

    private final double MASS;
    private final double GEAR_RATIO;
    private final double STOWED_ANGLE;
    private final double LOWER_ANGLE_LIMIT;
    private final double UPPER_ANGLE_LIMIT;
    private final MotorParameters MOTOR;

    /**
     * Constructs an instance of this class.
     * 
     * @param MOTOR The type of motor.
     * @param MASS  The mass of the arm in kilograms.
     */
    ArmParameters(double MASS, double GEAR_RATIO, double STOWED_ANGLE, double LOWER_ANGLE_LIMIT,
            double UPPER_ANGLE_LIMIT, MotorParameters MOTOR) {
        this.MASS = MASS;
        this.GEAR_RATIO = GEAR_RATIO;
        this.STOWED_ANGLE = STOWED_ANGLE;
        this.LOWER_ANGLE_LIMIT = LOWER_ANGLE_LIMIT;
        this.UPPER_ANGLE_LIMIT = UPPER_ANGLE_LIMIT;
        this.MOTOR = MOTOR;
    }

    public double getMass() {
        return MASS;
    }

    public double getGearRatio() {
        return GEAR_RATIO;
    }

    public double getStowedAngle() {
        return STOWED_ANGLE;
    }

    public double getLowerAngleLimit() {
        return LOWER_ANGLE_LIMIT;
    }

    public double getUpperAngleLimit() {
        return UPPER_ANGLE_LIMIT;
    }

    public MotorParameters getMotorType() {
        return MOTOR;
    }

    public double getMaxAngularSpeed(){
        return MOTOR.getFreeSpeedRPM() * (2 * Math.PI) / GEAR_RATIO / 60.0;
    }
    
    public double getMaxAngularAcceleration(){
        return (2 * MOTOR.getStallTorque() * GEAR_RATIO) / MASS;
    }

}