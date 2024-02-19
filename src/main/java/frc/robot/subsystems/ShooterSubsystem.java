/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
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

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private static final MotorParameters MOTOR = MotorParameters.NeoV1_1;

  private static final double GEAR_RATIO = 1.0;
  public static final double MAX_RPM = MOTOR.getFreeSpeedRPM() / GEAR_RATIO;
  private static final double ENCODER_CONVERSION_FACTOR = 1 / GEAR_RATIO;
  private static final double KS = 0.15;
  private static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_RPM;
  public static final double SPIN_FACTOR = 0.80;
  private static final double RPM_TOLERANCE = 50.0;

  private final CANSparkFlex leftMotor =
      new CANSparkFlex(RobotConstants.CAN.SparkMax.SHOOTER_LEFT_PORT, MotorType.kBrushless);
  private final CANSparkFlex rightMotor =
      new CANSparkFlex(RobotConstants.CAN.SparkMax.SHOOTER_RIGHT_PORT, MotorType.kBrushless);

  private double currentLeftRPM;
  private double currentRightRPM;
  private double goalRPM = 0;

  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private PIDController leftController = new PIDController(0.002088, 0, 0); // TODO assign value
  private PIDController rightController = new PIDController(0.002088, 0, 0); // TODO assign value
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);
  private boolean isEnabled = false;

  private BooleanLogEntry enabledLogger =
      new BooleanLogEntry(DataLogManager.getLog(), "/Shooter/Enabled");
  private DoubleLogEntry goalRPMLogger =
      new DoubleLogEntry(DataLogManager.getLog(), "/Shooter/Goal RPM");
  private DoubleLogEntry leftRPMLogger =
      new DoubleLogEntry(DataLogManager.getLog(), "/Shooter/Left RPM");
  private DoubleLogEntry rightRPMLogger =
      new DoubleLogEntry(DataLogManager.getLog(), "/Shooter/Right RPM");

  /** Creates ShooterSubsystem. */
  public ShooterSubsystem() {
    leftMotor.setIdleMode(IdleMode.kCoast);
    leftMotor.setInverted(true);
    rightMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setInverted(false);
    leftEncoder.setVelocityConversionFactor(ENCODER_CONVERSION_FACTOR);
    rightEncoder.setVelocityConversionFactor(ENCODER_CONVERSION_FACTOR);
    leftController.setTolerance(RPM_TOLERANCE);
    rightController.setTolerance(RPM_TOLERANCE);
  }

  private void setGoalRPMInternal(double goalRPM) {
    this.goalRPM = goalRPM;
    leftController.setSetpoint(goalRPM);
    rightController.setSetpoint(goalRPM * SPIN_FACTOR);
    goalRPMLogger.append(goalRPM);
  }

  public void setGoalRPM(double goalShooterRPM) {
    if (!isEnabled) {
      enabledLogger.append(true);
    }

    isEnabled = true;
    setGoalRPMInternal(goalShooterRPM);
  }

  public boolean atGoalRPM() {
    return leftController.atSetpoint();
  }

  /** Disables the Shooter PID controller and stops the motor. */
  public void disable() {
    if (isEnabled) {
      enabledLogger.append(false);
      goalRPMLogger.append(0);
    }

    isEnabled = false;
    stopMotor();
    leftController.reset();
    rightController.reset();
    goalRPM = 0;
  }

  /** Stops Shooter Motor. */
  public void stopMotor() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public double getRPM() {
    return currentLeftRPM;
  }

  public double getGoalRPM() {
    return goalRPM;
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

    currentLeftRPM = leftEncoder.getVelocity();
    currentRightRPM = rightEncoder.getVelocity();

    if (isEnabled) {
      double leftVoltage = feedforward.calculate(goalRPM);
      double rightVoltage = feedforward.calculate(goalRPM * SPIN_FACTOR);

      leftVoltage += leftController.calculate(currentLeftRPM);
      rightVoltage += rightController.calculate(currentRightRPM);

      setMotorVoltages(leftVoltage, rightVoltage);
    }

    leftRPMLogger.append(currentLeftRPM);
    rightRPMLogger.append(currentRightRPM);
  }

  public void addShuffleboardLayout(ShuffleboardTab tab) {
    ShuffleboardLayout shooterLayout =
        tab.getLayout("Shooter", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 3);

    shooterLayout.addDouble("Goal RPM", () -> goalRPM);
    shooterLayout.addDouble("Left RPM", () -> currentLeftRPM);
    shooterLayout.addDouble("Right RPM", () -> currentRightRPM);
    shooterLayout.addBoolean("Enabled", () -> isEnabled);
  }
}
