/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.parameters;

import static frc.robot.parameters.MotorParameters.NeoV1_1;

import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.motors.CANSparkMaxMotorAdapter;
import frc.robot.motors.MotorAdapter;

/** An enum representing the properties of the shooter for a specific robot instance. */
public enum ShooterParameters {
  PracticeShooter(NeoV1_1, 4, 10),
  ;
  private final MotorParameters motor;
  private final int leftDeviceID;
  private final int rightDeviceID;

  ShooterParameters(MotorParameters motor, int leftDeviceID, int rightDeviceID) {
    this.motor = motor;
    this.leftDeviceID = leftDeviceID;
    this.rightDeviceID = rightDeviceID;
  }

  /***
   * Creates a motor adapter with the specified motor.
   * @param motorParameters The motor type.
   * @param deviceID The CAN ID of the motor controller.
   * @return
   */
  private static MotorAdapter createMotorAdapter(MotorParameters motorParameters, int deviceID) {
    switch (motorParameters) {
      case NeoV1_1:
        return new CANSparkMaxMotorAdapter(deviceID, MotorType.kBrushless);

      default:
        throw new IllegalArgumentException("Invalid Motor Value");
    }
  }

  /***
   * Creates the left motor.
   * @return
   */
  public MotorAdapter createLeftMotor() {
    return createMotorAdapter(motor, leftDeviceID);
  }

  /***
   * Creates the right motor.
   * @return
   */
  public MotorAdapter createRighMotor() {
    return createMotorAdapter(motor, rightDeviceID);
  }
};
