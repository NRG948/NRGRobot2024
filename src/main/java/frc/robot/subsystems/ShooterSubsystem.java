/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.NoteCommands;
import frc.robot.motors.MotorandEncoderAdapter;
import frc.robot.parameters.ShooterParameters;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {
  @RobotPreferencesValue(column = 1, row = 0)
  public static final RobotPreferences.EnumValue<ShooterParameters> PARAMETERS =
      new RobotPreferences.EnumValue<ShooterParameters>(
          "Arm+Shooter", "Shooter", ShooterParameters.PracticeShooter);

  @RobotPreferencesValue(column = 1, row = 1)
  public static final RobotPreferences.DoubleValue LEFT_KP =
      new RobotPreferences.DoubleValue("Arm+Shooter", "Left Shooter kP", 0.004);

  @RobotPreferencesValue(column = 1, row = 2)
  public static final RobotPreferences.DoubleValue RIGHT_KP =
      new RobotPreferences.DoubleValue("Arm+Shooter", "Right Shooter kP", 0.003);

  @RobotPreferencesValue(column = 1, row = 3)
  public static final RobotPreferences.DoubleValue SPIN_FACTOR =
      new RobotPreferences.DoubleValue("Arm+Shooter", "Spin Factor", 0.85);

  private static final double RPM_TOLERANCE = 100.0;

  private final MotorandEncoderAdapter leftMotor = PARAMETERS.getValue().createLeftMotor();
  private final MotorandEncoderAdapter rightMotor = PARAMETERS.getValue().createRightMotor();

  private double currentLeftRPM;
  private double currentRightRPM;

  private Supplier<Rotation2d> orientationSupplier;

  private PIDController leftController =
      new PIDController(LEFT_KP.getValue(), 0, 0); // TODO assign value
  private PIDController rightController =
      new PIDController(RIGHT_KP.getValue(), 0, 0); // TODO assign value
  private SimpleMotorFeedforward feedforward = PARAMETERS.getValue().getFeedfoward();
  private boolean isEnabled = false;

  private BooleanLogEntry enabledLogger =
      new BooleanLogEntry(DataLogManager.getLog(), "/Shooter/Enabled");
  private DoubleLogEntry leftGoalRPMLogger =
      new DoubleLogEntry(DataLogManager.getLog(), "/Shooter/Left Goal RPM");
  private DoubleLogEntry rightGoalRPMLogger =
      new DoubleLogEntry(DataLogManager.getLog(), "/Shooter/Right Goal RPM");
  private DoubleLogEntry leftRPMLogger =
      new DoubleLogEntry(DataLogManager.getLog(), "/Shooter/Left RPM");
  private DoubleLogEntry rightRPMLogger =
      new DoubleLogEntry(DataLogManager.getLog(), "/Shooter/Right RPM");

  /** Creates ShooterSubsystem. */
  public ShooterSubsystem(Supplier<Rotation2d> orientationSupplier) {
    this.orientationSupplier = orientationSupplier;

    leftMotor.setBrakeMode(false);
    leftMotor.setInverted(true);
    rightMotor.setBrakeMode(false);
    rightMotor.setInverted(false);
    double velocityConversionFactor = 1.0 / PARAMETERS.getValue().getGearRatio();
    leftMotor.setVelocityConversionFactor(velocityConversionFactor);
    rightMotor.setVelocityConversionFactor(velocityConversionFactor);
    leftController.setTolerance(RPM_TOLERANCE);
    rightController.setTolerance(RPM_TOLERANCE);
  }

  private void setGoalRPMInternal(double goalRPM) {
    if (orientationSupplier.get().getRadians() > 0) {
      leftController.setSetpoint(goalRPM * SPIN_FACTOR.getValue());
      rightController.setSetpoint(goalRPM);
    } else {
      leftController.setSetpoint(goalRPM);
      rightController.setSetpoint(goalRPM * SPIN_FACTOR.getValue());
    }
    leftController.setPID(LEFT_KP.getValue(), 0, 0);
    rightController.setPID(RIGHT_KP.getValue(), 0, 0);
    leftGoalRPMLogger.append(leftController.getSetpoint());
    rightGoalRPMLogger.append(rightController.getSetpoint());
  }

  public void setGoalRPM(double goalShooterRPM) {
    if (!isEnabled) {
      enabledLogger.append(true);
      isEnabled = true;
    }
    setGoalRPMInternal(goalShooterRPM);
  }

  public boolean atGoalRPM() {
    return rightController.atSetpoint() && leftController.atSetpoint();
  }

  /** Disables the Shooter PID controller and stops the motor. */
  public void disable() {
    if (isEnabled) {
      enabledLogger.append(false);
      leftGoalRPMLogger.append(0);
    }

    isEnabled = false;
    stopMotor();
    leftController.reset();
    rightController.reset();
  }

  /** Stops Shooter Motor. */
  public void stopMotor() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public double getRPM() {
    return currentLeftRPM;
  }

  public boolean isEnabled() {
    return isEnabled;
  }

  public void setMotorVoltages(double leftVoltage, double rightVoltage) {
    leftMotor.setVoltage(leftVoltage);
    rightMotor.setVoltage(rightVoltage);
  }

  @Override
  public void periodic() {

    currentLeftRPM = leftMotor.getVelocity();
    currentRightRPM = rightMotor.getVelocity();

    if (isEnabled) {
      double leftVoltage = feedforward.calculate(leftController.getSetpoint());
      double rightVoltage = feedforward.calculate(rightController.getSetpoint());

      leftVoltage += leftController.calculate(currentLeftRPM);
      rightVoltage += rightController.calculate(currentRightRPM);

      setMotorVoltages(leftVoltage, rightVoltage);
    }

    leftRPMLogger.append(currentLeftRPM);
    rightRPMLogger.append(currentRightRPM);
  }

  public void addShuffleboardLayout(ShuffleboardTab tab, Subsystems subsystems) {
    ShuffleboardLayout shooterLayout =
        tab.getLayout("Shooter", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 5))
            .withPosition(4, 0)
            .withSize(2, 4);

    shooterLayout.addDouble("Goal Left RPM", () -> leftController.getSetpoint()).withPosition(0, 0);
    shooterLayout
        .addDouble("Goal Right RPM", () -> rightController.getSetpoint())
        .withPosition(1, 0);
    shooterLayout.addDouble("Left RPM", () -> currentLeftRPM).withPosition(0, 1);
    shooterLayout.addDouble("Right RPM", () -> currentRightRPM).withPosition(1, 1);
    shooterLayout.addBoolean("Enabled", () -> isEnabled).withPosition(0, 2);

    GenericEntry rpmEntry = shooterLayout.add("RPM", 0).withPosition(0, 3).getEntry();
    shooterLayout
        .add(
            "Shoot",
            Commands.defer(
                () -> {
                  double rpm = rpmEntry.getDouble(100);
                  return Commands.sequence(
                      Commands.print("SHOOT AT " + rpm), // ShooterCommands.setRPM(subsystems, rpm)
                      NoteCommands.shoot(subsystems, rpm));
                },
                Set.of(subsystems.shooter, subsystems.indexer)))
        .withPosition(0, 4);
    shooterLayout
        .add(
            "Cancel",
            Commands.race( // should this be parallel?
                Commands.runOnce(subsystems.indexer::disable, subsystems.indexer),
                Commands.runOnce(subsystems.shooter::disable, subsystems.shooter)))
        .withPosition(1, 4);
  }
}
