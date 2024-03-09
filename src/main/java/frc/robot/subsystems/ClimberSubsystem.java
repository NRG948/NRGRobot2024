// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  @RobotPreferencesValue()
  public static final RobotPreferences.DoubleValue WIND_VOLTAGE = new RobotPreferences.DoubleValue("Climber",
      "Winch Wind Voltage", 3.0);

  @RobotPreferencesValue()
  public static final RobotPreferences.DoubleValue UNWIND_VOLTAGE = new RobotPreferences.DoubleValue("Climber",
      "Winch Unwind Voltage", 1.5);

  private final TalonFX winchLeftMotor = new TalonFX(155); // TODO: update id
  private final TalonFX winchRightMotor = new TalonFX(156); // TODO: update id

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    winchLeftMotor.setInverted(true);
  }

  public void setVoltage(double voltage) {
    winchLeftMotor.setVoltage(voltage);
    winchRightMotor.setVoltage(voltage);
  }

  public void wind() {
    setVoltage(WIND_VOLTAGE.getValue());
  }

  public void unwind() {
    setVoltage(UNWIND_VOLTAGE.getValue());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
