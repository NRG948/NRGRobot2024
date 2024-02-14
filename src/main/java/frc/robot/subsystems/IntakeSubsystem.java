// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.CAN;
import frc.robot.parameters.MotorParameters;

/**
 * The intake subsystem is responsible for acquiring game elements from the
 * floor.
 */
public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkFlex motor = new CANSparkFlex(CAN.SparkMax.INTAKE_PORT, MotorType.kBrushless);
  private boolean isEnabled = false;
  private double goalRPM;

  public static double GEAR_RATIO = 1.0; // TODO: get gear ratio once mech built

  public static double MAX_RPM = MotorParameters.NeoV1_1.getFreeSpeedRPM() / GEAR_RATIO;
  public static double MAX_ACCELERATION = (2 * MotorParameters.NeoV1_1.getStallTorque() * GEAR_RATIO)
      / RobotConstants.INDEXER_MASS;

  public static double KS = 0.15;
  public static double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_RPM;
  public static double KA = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ACCELERATION;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV, KA);


  /** Creates a
   *  new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(true);
  }

  /**
   * Runs the motor
   * 
   * @param power
   */
  public void runMotor(double power) {
    motor.set(power);
  }

  /**
   * Stops the motor
   */
  public void stopMotor() {
    motor.stopMotor();
  }

  /**
   * Enable the intake inwards.
   */
  public void in() {
    isEnabled = true;
    goalRPM = IndexerSubsystem.INDEXER_INTAKE_RPM.getValue();
  }

  /**
   * Enable the intake outward.
   */
  public void out() {
    isEnabled = true;
    goalRPM = -IndexerSubsystem.INDEXER_INTAKE_RPM.getValue();
  }

  /**
   * Disable the intake.
   */
  public void disable() {
    isEnabled = false;
    stopMotor();
  }

  @Override
  public void periodic() {
    if (isEnabled) {
      double feedforward = this.feedforward.calculate(goalRPM);
      motor.setVoltage(feedforward);
    }
  }
}
