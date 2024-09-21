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
import frc.robot.parameters.IndexerParameters;

@RobotPreferencesLayout(groupName = "Indexer+Intake", column = 6, row = 0, width = 1, height = 4)
public class IndexerSubsystem extends SubsystemBase {

  @RobotPreferencesValue
  public static RobotPreferences.EnumValue<IndexerParameters> PARAMETERS =
      new RobotPreferences.EnumValue<IndexerParameters>(
          "Indexer+Intake", "Indexer", IndexerParameters.PracticeBase2024);

  public static final double GEAR_RATIO = PARAMETERS.getValue().getGearRatio();
  public static final double INDEXER_DIAMETER = PARAMETERS.getValue().getDiameter();
  public static double ENCODER_CONVERSION_FACTOR = (Math.PI * INDEXER_DIAMETER) / GEAR_RATIO;

  public static double MAX_VELOCITY =
      (PARAMETERS.getValue().getMotorParameters().getFreeSpeedRPM() * Math.PI * INDEXER_DIAMETER)
          / (GEAR_RATIO * 60);
  public static double MAX_ACCELERATION =
      (2
              * PARAMETERS.getValue().getMotorParameters().getStallTorque()
              * GEAR_RATIO
              * Math.PI
              * INDEXER_DIAMETER)
          / PARAMETERS.getValue().getMass();

  public static double KS = 0.15;
  public static double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;
  public static double KA = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ACCELERATION;

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Indexer+Intake", "Enable Tab", false);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue FEED_VELOCITY =
      new RobotPreferences.DoubleValue("Indexer+Intake", "Indexer Feed Velocity", 3.0);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue AMP_OUTAKE_VELOCITY =
      new RobotPreferences.DoubleValue("Indexer+Intake", "Amp Outake Velocity", 3.0);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue AUTO_CENTER_VELOCITY =
      new RobotPreferences.DoubleValue("Indexer+Intake", "Auto Center Velocity", 2.0);

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue HAS_LOWER_BEAM_BREAK =
      new RobotPreferences.BooleanValue("Indexer+Intake", "Has Lower Beam Break", true);

  private boolean noteBreakingPracticeBeam = false;
  private boolean isEnabled = false;
  private double goalVelocity = 0;
  private double currentVelocity = 0;

  private final CANSparkMax motor =
      new CANSparkMax(RobotConstants.CAN.SparkMax.INDEXER_PORT, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkLimitSwitch upperBeamBreak = motor.getForwardLimitSwitch(Type.kNormallyOpen);
  private final SparkLimitSwitch lowerBeamBreak = motor.getReverseLimitSwitch(Type.kNormallyOpen);
  private final SimpleMotorFeedforward indexerFeedfoward = new SimpleMotorFeedforward(KS, KV, KA);

  private final BooleanLogEntry noteAtShootPositionLogger =
      new BooleanLogEntry(DataLogManager.getLog(), "Indexer/Note At Shoot Position");
  private final BooleanLogEntry noteAtEntryLogger =
      new BooleanLogEntry(DataLogManager.getLog(), "Indexer/Note At Entry");
  private final DoubleLogEntry goalVelocityLogger =
      new DoubleLogEntry(DataLogManager.getLog(), "Indexer/Goal Velocity");

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(PARAMETERS.getValue().getInverted());
    encoder.setVelocityConversionFactor(ENCODER_CONVERSION_FACTOR);
    encoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR);
    upperBeamBreak.enableLimitSwitch(false);
    lowerBeamBreak.enableLimitSwitch(false);
  }

  /**
   * Returns whether the note is at the shoot position (upper beam break is broken).
   *
   * @return
   */
  public boolean isNoteBreakingPracticeBeam() {
    return noteBreakingPracticeBeam;
  }

  // /**
  //  * Returns whether the note is at the entry of the indexer (lower beam break is broken).
  //  *
  //  * @return
  //  */
  // public boolean isNoteBreakingLowerBeam() {
  //   return noteBreakingLowerBeam;
  // }

  // public boolean isNoteBreakingEitherBeam() {
  //   return noteBreakingUpperBeam || noteBreakingLowerBeam;
  // }

  public void feed() {
    intake(FEED_VELOCITY.getValue());
  }

  public void intake() {
    intake(IntakeSubsystem.INTAKE_VELOCITY.getValue());
  }

  public void outtake() {
    outtake(IntakeSubsystem.INTAKE_VELOCITY.getValue());
  }

  public void intake(double velocity) {
    isEnabled = true;
    goalVelocity = velocity;
  }

  /** Outtake at the absolute given velocity. */
  public void outtake(double velocity) {
    isEnabled = true;
    goalVelocity = -Math.abs(velocity);
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
    boolean noteBreakingUpperBeam = upperBeamBreak.isPressed();
    if (this.noteBreakingPracticeBeam != noteBreakingUpperBeam) {
      noteAtShootPositionLogger.append(noteBreakingUpperBeam);
      this.noteBreakingPracticeBeam = noteBreakingUpperBeam;
    }

    // boolean noteBreakingLowerBeam = lowerBeamBreak.isPressed();
    // if (this.noteBreakingLowerBeam != noteBreakingLowerBeam) {
    //   noteAtEntryLogger.append(noteBreakingLowerBeam);
    //   this.noteBreakingLowerBeam = noteBreakingLowerBeam;
    // }

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
    layout.addBoolean("Note Detected", () -> noteBreakingPracticeBeam);
    // layout.addBoolean("Lower Note Detected", () -> noteBreakingLowerBeam);
  }
}
