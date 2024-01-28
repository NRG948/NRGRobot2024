// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences.DoubleValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The intake subsystem is responsible for acquiring game elements from the
 * floor.
 */
public class IntakeSubsystem extends SubsystemBase {
  private static final double INTAKE_POWER = 0.3; // moving this 
  private final static CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
  
  private static double KS = 0.15;
  public final DoubleValue KV = new DoubleValue("IntakeSubsystem", "KV", 1.0);
  public final DoubleValue KA = new DoubleValue("IntakeSubsystem", "KA", 1.0);
  private boolean isEnabled = false;
  private double motorPower;
  private final SimpleMotorFeedforward indexerFeedforward = new SimpleMotorFeedforward(KS, KV.getValue(), KA.getValue());

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor.setIdleMode(IdleMode.kBrake);
  }
  
  public void runMotor(double power) {
    double voltage  = indexerFeedforward.calculate(power);
    motor.set(voltage);
  }
  
  public void stopMotor() {
    motor.stopMotor();
  }
  
  /**
   * Enable the intake upward.
   */
  public void up() {
    isEnabled = true;
    motorPower = INTAKE_POWER;
  }
  
  /**
   * Enable the intake downward.
   */
  public void down() {
    isEnabled = true;
    motorPower = -INTAKE_POWER;
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
      runMotor(motorPower);
    }
  }
}