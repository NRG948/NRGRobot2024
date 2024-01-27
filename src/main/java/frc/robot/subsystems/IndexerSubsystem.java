// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN.SparkMax;

@RobotPreferencesLayout(groupName = "Indexer", column = 0, row = 1, width = 2, height = 3)
public class IndexerSubsystem extends SubsystemBase {

  @RobotPreferencesValue
  private final RobotPreferences.DoubleValue KS = new RobotPreferences.DoubleValue("Indexer", "KS", 1.0);
  private final RobotPreferences.DoubleValue KV = new RobotPreferences.DoubleValue("Indexer", "KV", 1.0);
  private final RobotPreferences.DoubleValue KA = new RobotPreferences.DoubleValue("Indexer", "KA", 1.0);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue INDEXER_FEED_RPM = new RobotPreferences.DoubleValue("Indexer",
      "Indexer Feed RPM", 1000.0);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue INDEXER_INTAKE_RPM = new RobotPreferences.DoubleValue("Indexer",
      "Indexer Intake RPM", 250.0);

  private boolean noteDetected = false;
  private boolean isEnabled = false;
  private double goalRPM = 0;
  
  private CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless); // TODO Get the device ID
  private final DigitalInput beamBreak = new DigitalInput(0); // TODO Get the channel
  private final SimpleMotorFeedforward indexerFeedfoward = new SimpleMotorFeedforward(KS.getValue(), KV.getValue(),
      KA.getValue());
  
  private final BooleanLogEntry noteDetectedLogger = new BooleanLogEntry(DataLogManager.getLog(), "Indexer Note Detector");
  private final DoubleLogEntry goalRPMLogger = new DoubleLogEntry(DataLogManager.getLog(), "Goal RPM Logger");
      

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    motor.setIdleMode(IdleMode.kBrake);
  }

  public boolean isNoteDetected() {
    return noteDetected;
  }

  public void feed() {
    isEnabled = true;
    goalRPM = INDEXER_FEED_RPM.getValue();
  }

  public void intake() {
    isEnabled = true;
    goalRPM = INDEXER_INTAKE_RPM.getValue();
  }

  public void outake() {
    isEnabled = true;
    goalRPM = -INDEXER_INTAKE_RPM.getValue();
  }

  public void disable() {
    isEnabled = false;
    goalRPM = 0;
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
      double voltage = indexerFeedfoward.calculate(goalRPM);
      goalRPMLogger.append(goalRPM);
      motor.setVoltage(voltage);
    }

  }
}
