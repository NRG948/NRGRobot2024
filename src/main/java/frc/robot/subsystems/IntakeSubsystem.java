// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  private final RelativeEncoder encoder = motor.getEncoder();
  private boolean isEnabled = false;
  private double goalVelocity;
  private double currentVelocity;

  public static double GEAR_RATIO = 3*26/24; 
  public static double INTAKE_DIAMETER = 0.036; // Diameter in meters
  public static double ENCODER_CONVERSION_FACTOR = (Math.PI * INTAKE_DIAMETER) / GEAR_RATIO;

  public static double MAX_VELOCITY = (MotorParameters.NeoV1_1.getFreeSpeedRPM() * Math.PI * INTAKE_DIAMETER) / (GEAR_RATIO * 60);
  public static double MAX_ACCELERATION = (2 * MotorParameters.NeoV1_1.getStallTorque() * GEAR_RATIO * Math.PI * INTAKE_DIAMETER) 
      / RobotConstants.INDEXER_MASS;

  public static double KS = 0.15;
  public static double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;
  public static double KA = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ACCELERATION;

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue INTAKE_VELOCITY = new RobotPreferences.DoubleValue("Indexer+Intake",
      "Indexer Intake Velocity", 0.2);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV, KA);



  /** Creates a
   *  new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(true);
    encoder.setVelocityConversionFactor(ENCODER_CONVERSION_FACTOR);
    encoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR);
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
    goalVelocity = IntakeSubsystem.INTAKE_VELOCITY.getValue();
  }

  /**
   * Enable the intake outward.
   */
  public void out() {
    isEnabled = true;
    goalVelocity = -IntakeSubsystem.INTAKE_VELOCITY.getValue();
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
    currentVelocity = encoder.getVelocity();
    if (isEnabled) {
      double feedforward = this.feedforward.calculate(goalVelocity);
      motor.setVoltage(feedforward);
    }
  }

  public void addShuffleboardLayout(ShuffleboardTab tab) {
    ShuffleboardLayout layout = tab.getLayout("Intake", BuiltInLayouts.kList)
      .withPosition(0, 0)
      .withSize(2, 3);
    layout.addDouble("Goal Velocity", () -> goalVelocity);
    layout.addDouble("Current Velocity", () -> currentVelocity);
    layout.addBoolean("Enabled", () -> isEnabled);
  }
}
