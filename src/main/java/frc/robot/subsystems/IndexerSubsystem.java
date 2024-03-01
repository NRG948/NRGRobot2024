/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.parameters.MotorParameters;

@RobotPreferencesLayout(groupName = "Indexer+Intake", column = 0, row = 1, width = 1, height = 2)
public class IndexerSubsystem extends SubsystemBase {

  public static double GEAR_RATIO = 3 * 26 / 15;
  public static double INDEXER_DIAMETER = 0.033; // Diameter in meters
  public static double ENCODER_CONVERSION_FACTOR = (Math.PI * INDEXER_DIAMETER) / GEAR_RATIO;

  public static double MAX_VELOCITY =
      (MotorParameters.NeoV1_1.getFreeSpeedRPM() * Math.PI * INDEXER_DIAMETER) / (GEAR_RATIO * 60);
  public static double MAX_ACCELERATION =
      (2 * MotorParameters.NeoV1_1.getStallTorque() * GEAR_RATIO * Math.PI * INDEXER_DIAMETER)
          / RobotConstants.INDEXER_MASS;

  public static double KS = 0.15;
  public static double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;
  public static double KA = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ACCELERATION;

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Indexer+Intake", "Enable Tab", false);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue FEED_VELOCITY =
      new RobotPreferences.DoubleValue("Indexer+Intake", "Indexer Feed Velocity", 3.0);

  private boolean noteDetected = false;
  private boolean isEnabled = false;
  private double goalVelocity = 0;
  private double currentVelocity = 0;

  private final CANSparkMax motor =
      new CANSparkMax(RobotConstants.CAN.SparkMax.INDEXER_PORT, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkLimitSwitch beamBreak = motor.getForwardLimitSwitch(Type.kNormallyOpen);
  private final SimpleMotorFeedforward indexerFeedfoward = new SimpleMotorFeedforward(KS, KV, KA);

  private final BooleanLogEntry noteDetectedLogger =
      new BooleanLogEntry(DataLogManager.getLog(), "Indexer Note Detector");
  private final DoubleLogEntry goalVelocityLogger =
      new DoubleLogEntry(DataLogManager.getLog(), "Indexer/Goal Velocity");

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue AMP_OUTAKE_VELOCITY =
      new RobotPreferences.DoubleValue("Indexer+Intake", "Amp Outake Velocity", 3.0);

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(true);
    encoder.setVelocityConversionFactor(ENCODER_CONVERSION_FACTOR);
    encoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR);
    beamBreak.enableLimitSwitch(false);
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

  public void outtakeToAmp() {
    isEnabled = true;
    goalVelocity = -IndexerSubsystem.AMP_OUTAKE_VELOCITY.getValue();
  }

  public void disable() {
    isEnabled = false;
    goalVelocity = 0;
    motor.stopMotor();
  }

  public void setMotorVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean noteDetected = beamBreak.isPressed();
    if (this.noteDetected != noteDetected) {
      noteDetectedLogger.append(noteDetected);
      this.noteDetected = noteDetected;
    }

    currentVelocity = encoder.getVelocity();

    if (isEnabled) {
      double voltage = indexerFeedfoward.calculate(goalVelocity);
      goalVelocityLogger.append(goalVelocity);
      motor.setVoltage(voltage);
    }
  }

  public void setBrakeMode(boolean brakeMode) {
    IdleMode idleMode;

    if (brakeMode) {
      idleMode = IdleMode.kBrake;
    } else {
      idleMode = IdleMode.kCoast;
    }
    motor.setIdleMode(idleMode);
  }

  public void addShuffleboardLayout(ShuffleboardTab tab) {
    ShuffleboardLayout layout =
        tab.getLayout("Indexer", BuiltInLayouts.kList).withSize(2, 3).withPosition(2, 0);
    layout.addDouble("Goal Velocity", () -> goalVelocity);
    layout.addDouble("Current Velocity", () -> currentVelocity);
    layout.addBoolean("Enabled", () -> isEnabled);
    layout.addBoolean("Note Detected", () -> noteDetected);
  }
}
