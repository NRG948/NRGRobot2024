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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.parameters.MotorParameters;

@RobotPreferencesLayout(groupName = "Arm+Shooter", row = 0, column = 2, width = 2, height = 1)
public class ArmSubsystem extends SubsystemBase {
  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Arm+Shooter", "Enable Tab", false);

  public static final double GEAR_RATIO = 168.0;
  public static final double MASS = 0.5; // TODO determine actual arm mass
  public static final double RADIANS_PER_REVOLUTION = (2 * Math.PI) / GEAR_RATIO;
  public static final MotorParameters MOTOR = MotorParameters.NeoV1_1;
  public static final double MAX_ANGULAR_SPEED =
      MOTOR.getFreeSpeedRPM() * RADIANS_PER_REVOLUTION / 60.0;
  public static final double MAX_ANGULAR_ACCELERATION =
      (2 * MOTOR.getStallTorque() * GEAR_RATIO) / MASS;
  public static final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * 0.5, MAX_ANGULAR_ACCELERATION);
  public static final double KS = 3.0;
  public static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_SPEED;
  public static final double KA =
      (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_ACCELERATION;
  public static final double KG = 0; // KA * 9.81;
  public static final double STOWED_ANGLE = -31.3;
  public static final double ARM_RADIANS_PER_MOTOR_ROTATION = (2 * Math.PI) / GEAR_RATIO;
  private static final double LOWER_ANGLE_LIMIT = Math.toRadians(-27);
  private static final double UPPER_ANGLE_LIMIT = Math.toRadians(80);

  private final CANSparkMax leftMotor =
      new CANSparkMax(RobotConstants.CAN.SparkMax.ARM_LEFT_PORT, MotorType.kBrushless);
  private final CANSparkMax rightMotor =
      new CANSparkMax(RobotConstants.CAN.SparkMax.ARM_RIGHT_PORT, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final DutyCycleEncoder absoluteEncoder =
      new DutyCycleEncoder(Constants.RobotConstants.DigitalIO.ARM_ABSOLUTE_ENCODER);

  private double rawAngleOffset = Math.toRadians(9.15);

  private double currentAngle = STOWED_ANGLE;

  private final ArmFeedforward feedForward = new ArmFeedforward(KS, KV, KA, KG);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(35.0, 0.0, 0.0, CONSTRAINTS);
  private boolean enablePeriodicControl = false;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    rightMotor.follow(leftMotor, true);
    leftMotor.setIdleMode(IdleMode.kCoast); // TODO kBreak
    rightMotor.setIdleMode(IdleMode.kCoast); // TODO kBreak
    leftEncoder.setPositionConversionFactor(ARM_RADIANS_PER_MOTOR_ROTATION);
    leftEncoder.setVelocityConversionFactor(ARM_RADIANS_PER_MOTOR_ROTATION);
    absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    absoluteEncoder.setDistancePerRotation(2 * Math.PI);
    controller.enableContinuousInput(-Math.PI, Math.PI);

    System.out.println("Arm max velocity: " + Math.toDegrees(MAX_ANGULAR_SPEED));
    System.out.println("Arm max accel: " + Math.toDegrees(MAX_ANGULAR_ACCELERATION));
  }

  /**
   * Sets the goal angle and enables periodic control.
   *
   * @param goalAngle The goal angle in radians.
   */
  public void setGoalAngle(double goalAngle) {
    controller.reset(currentAngle);
    controller.setGoal(goalAngle);
    enablePeriodicControl = true;
  }

  /** Disables periodic control. */
  public void disable() {
    enablePeriodicControl = false;
    leftMotor.stopMotor();
  }

  /** Returns whether the arm is at the goal angle. */
  public boolean atGoalAngle() {
    return controller.atGoal();
  }

  /**
   * Returns the current angle of the arm.
   *
   * @return The current angle of the arm.
   */
  public double getAngle() {
    return currentAngle;
  }

  /**
   * Sets the same voltage for both motors.
   *
   * @param voltage The desired voltage.
   */
  public void setMotorVoltages(double voltage) {
    if (voltage < 0 && currentAngle < LOWER_ANGLE_LIMIT) {
      voltage = 0;
    } else if (voltage > 0 && currentAngle > UPPER_ANGLE_LIMIT) {
      voltage = 0;
    }

    leftMotor.setVoltage(voltage);
  }

  /**
   * Sets the same power for both motors.
   *
   * @param powerPercent The desired power. Range from -1.0 to 1.0.
   */
  public void setMotorPowers(double powerPercent) {
    powerPercent = MathUtil.clamp(powerPercent, -1.0, 1.0);
    setMotorVoltages(powerPercent * RobotController.getBatteryVoltage());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentAngle = MathUtil.angleModulus(rawAngleOffset - absoluteEncoder.getDistance());

    if (enablePeriodicControl) {
      double feedback = controller.calculate(currentAngle);
      double feedForward =
          this.feedForward.calculate(currentAngle, controller.getSetpoint().velocity);
      setMotorVoltages(feedForward + feedback);
    }
  }

  public void addShuffleboardLayout(ShuffleboardTab tab) {
    ShuffleboardLayout infolayout =
        tab.getLayout("Arm Info", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    infolayout.addDouble("Current Angle", () -> Math.toDegrees(getAngle()));
    infolayout.addDouble("Goal Angle", () -> Math.toDegrees(controller.getGoal().position));
  }
}
