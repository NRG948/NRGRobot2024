/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.parameters;

import static frc.robot.parameters.MotorParameters.KrakenX60;
import static frc.robot.parameters.MotorParameters.NeoV1_1;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.RobotConstants;
import frc.robot.motors.CANSparkMaxAdapter;
import frc.robot.motors.CANTalonFXAdapter;
import frc.robot.motors.MotorandEncoderAdapter;

/** An enum representing the properties of the shooter for a specific robot instance. */
public enum ShooterParameters {
  PracticeShooter(NeoV1_1, 4, 10, 1.0, 0.9, 0.15),
  CompetitionShooter(KrakenX60, 4, 15, 1.0, 0.9, 0.15) // TODO update for comp bot
;
  private final MotorParameters motor;
  private final int leftDeviceID;
  private final int rightDeviceID;
  private final double maxRPM;
  private final double gearRatio;
  private final FeedforwardConstants feedforward;

  /**
   * Constructs an instance of this enum.
   *
   * @param motor The shooter motor type.
   * @param leftDeviceID The CAN id of left motor.
   * @param rightDeviceID The CAN id of right motor.
   * @param gearRatio The gear ratio.
   * @param efficiency The estimated efficiency of system.
   * @param kS The kS feedforward control constant.
   */
  ShooterParameters(
      MotorParameters motor,
      int leftDeviceID,
      int rightDeviceID,
      double gearRatio,
      double efficiency,
      double kS) {
    this.motor = motor;
    this.leftDeviceID = leftDeviceID;
    this.rightDeviceID = rightDeviceID;
    this.gearRatio = gearRatio;

    maxRPM = (motor.getFreeSpeedRPM() / gearRatio) * efficiency;
    feedforward =
        new FeedforwardConstants(kS, (RobotConstants.MAX_BATTERY_VOLTAGE - kS) / maxRPM, 0);
  }

  /***
   * Creates a motor adapter with the specified motor.
   * @param motorParameters The motor type.
   * @param deviceID The CAN ID of the motor controller.
   * @return
   */
  private static MotorandEncoderAdapter createMotorAdapter(
      MotorParameters motorParameters, int deviceID) {
    switch (motorParameters) {
      case NeoV1_1:
        return new CANSparkMaxAdapter(deviceID, MotorType.kBrushless);

      case KrakenX60:
        return new CANTalonFXAdapter(deviceID);

      default:
        throw new IllegalArgumentException("Invalid Motor Value");
    }
  }

  /***
   * Creates the left motor.
   * @return
   */
  public MotorandEncoderAdapter createLeftMotor() {
    return createMotorAdapter(motor, leftDeviceID);
  }

  /***
   * Creates the right motor.
   * @return
   */
  public MotorandEncoderAdapter createRightMotor() {
    return createMotorAdapter(motor, rightDeviceID);
  }

  /**
   * Returns maximum RPM.
   *
   * @return
   */
  public double getMaxRPM() {
    return maxRPM;
  }

  /**
   * Returns Gear Ratio.
   *
   * @return
   */
  public double getGearRatio() {
    return gearRatio;
  }

  /**
   * Returns SimpleMotorFeedForward object for shooter.
   *
   * @return
   */
  public SimpleMotorFeedforward getFeedfoward() {
    return new SimpleMotorFeedforward(feedforward.kS, feedforward.kV);
  }

  /**
   * Returns kS feedfoward control constant.
   *
   * @return
   */
  public double getKS() {
    return feedforward.kS;
  }

  /**
   * Returns kV feedfoward control constant.
   *
   * @return
   */
  public double getKV() {
    return feedforward.kV;
  }
};
