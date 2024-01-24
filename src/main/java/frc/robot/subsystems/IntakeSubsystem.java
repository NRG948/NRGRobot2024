// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;

/**
 * The intake subsystem is responsible for acquiring game elements from the
 * floor.
 */
public class IntakeSubsystem extends SubsystemBase {
  private static final double INTAKE_POWER = 0.3; // moving this

  private final CANSparkFlex motor = new CANSparkFlex(CAN.SparkMax.INTAKE_PORT, MotorType.kBrushless);
  private final DigitalInput beamBreak = new DigitalInput(0);
  private boolean isEnabled = false;
  private double motorPower;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor.setIdleMode(IdleMode.kBrake);
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
    motorPower = INTAKE_POWER;
  }

  /**
   * Enable the intake outward.
   */
  public void out() {
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

  /**
   * Returns true when the note is detected in the intake.
   */
  public boolean isNoteDetected(){
    return !beamBreak.get();
  }

  @Override
  public void periodic() {
    if (isEnabled) {
      runMotor(motorPower);
    }
  }
}
