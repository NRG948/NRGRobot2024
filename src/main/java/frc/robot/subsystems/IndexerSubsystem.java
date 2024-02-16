// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.parameters.MotorParameters;

@RobotPreferencesLayout(groupName = "Indexer", column = 0, row = 1, width = 2, height = 3)
public class IndexerSubsystem extends SubsystemBase {

  public static double GEAR_RATIO = 3*26/15;
  public static double INDEXER_DIAMETER = 0.033; // Diameter in meters

  public static double MAX_VELOCITY = (MotorParameters.NeoV1_1.getFreeSpeedRPM() * Math.PI * INDEXER_DIAMETER)/ (GEAR_RATIO * 60);
  public static double MAX_ACCELERATION = (2 * MotorParameters.NeoV1_1.getStallTorque() * GEAR_RATIO * Math.PI * INDEXER_DIAMETER)
      / RobotConstants.INDEXER_MASS;

  public static double KS = 0.15;
  public static double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;
  public static double KA = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ACCELERATION;

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue FEED_VELOCITY = new RobotPreferences.DoubleValue("Indexer",
      "Indexer Feed Velocity", 0.8 * MAX_VELOCITY);

  private boolean noteDetected = false;
  private boolean isEnabled = false;
  private double goalVelocity = 0;

  private CANSparkMax motor = new CANSparkMax(RobotConstants.CAN.SparkMax.INDEXER_PORT, MotorType.kBrushless);
  private final DigitalInput beamBreak = new DigitalInput(RobotConstants.DigitalIO.INDEXER_BEAM_BREAK);
  private final SimpleMotorFeedforward indexerFeedfoward = new SimpleMotorFeedforward(KS, KV, KA);

  private final BooleanLogEntry noteDetectedLogger = new BooleanLogEntry(DataLogManager.getLog(),
      "Indexer Note Detector");
  private final DoubleLogEntry goalVelocityLogger = new DoubleLogEntry(DataLogManager.getLog(), "Indexer/Goal Velocity");

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    motor.setIdleMode(IdleMode.kBrake);
  }

  public boolean isNoteDetected() {
    return noteDetected;
  }

  public void feed() {
    isEnabled = true;
    goalVelocity = FEED_VELOCITY.getValue();
  }

  public void intake() {
    isEnabled = true;
    goalVelocity = IntakeSubsystem.INTAKE_VELOCITY.getValue();
  }

  public void outtake() {
    isEnabled = true;
    goalVelocity = -IntakeSubsystem.INTAKE_VELOCITY.getValue();
  }

  public void disable() {
    isEnabled = false;
    goalVelocity = 0;
    motor.stopMotor();
  }

  public void runMotor(double power) {
    motor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean noteDetected = !beamBreak.get();
    if (this.noteDetected != noteDetected) {
      noteDetectedLogger.append(noteDetected);
      this.noteDetected = noteDetected;
    }

    if (isEnabled) {
      double voltage = indexerFeedfoward.calculate(goalVelocity);
      goalVelocityLogger.append(goalVelocity);
      motor.setVoltage(voltage);
    }

  }
}
