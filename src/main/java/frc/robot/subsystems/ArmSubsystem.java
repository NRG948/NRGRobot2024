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
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.parameters.MotorParameters;

@RobotPreferencesLayout(groupName = "Arm", row = 3, column = 4, width = 2, height = 4)
public class ArmSubsystem extends SubsystemBase {
  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue enableTab = new RobotPreferences.BooleanValue(
      "Arm", "Enable Tab", false);

  public static final double GEAR_RATIO = 168.0;
  public static final double MASS = 125.0; // TODO determine actual arm mass
  public static final double RADIANS_PER_REVOLUTION = (2 * Math.PI) / GEAR_RATIO;
  public static final MotorParameters MOTOR = MotorParameters.NeoV1_1;
  public static final double MAX_ANGULAR_SPEED = MOTOR.getFreeSpeedRPM() * RADIANS_PER_REVOLUTION / 60.0;
  public static final double MAX_ANGULAR_ACCELERATION = (2 * MOTOR.getStallTorque() * GEAR_RATIO) / MASS;
  public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_SPEED * 0.5, MAX_ANGULAR_ACCELERATION);
  public static final double KS = 3.0;
  public static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_SPEED;
  public static final double KA = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_ACCELERATION;
  public static final double KG = KA * 9.81;
  public static final double STOWED_ANGLE = -31.3;
  public static final double ARM_RADIANS_PER_MOTOR_ROTATION = (2 * Math.PI) / GEAR_RATIO;

  private final CANSparkMax leftMotor = new CANSparkMax(RobotConstants.CAN.SparkMax.ARM_LEFT_PORT,
      MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(RobotConstants.CAN.SparkMax.ARM_RIGHT_PORT,
      MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(Constants.RobotConstants.DigitalIO.ARM_ABSOLUTE_ENCODER);

  private double rawAngleOffset = Math.toRadians(9.15);

  private double currentAngle = STOWED_ANGLE;

  private final ArmFeedforward feedForward = new ArmFeedforward(KS, KV, KA, KG);
  private final Timer timer = new Timer();
  private final ProfiledPIDController controller = new ProfiledPIDController(1.0, 0.0, 0.0, CONSTRAINTS);
  private double goalAngle;
  private boolean enablePeriodicControl = false;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    rightMotor.follow(leftMotor, true);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftEncoder.setPositionConversionFactor(ARM_RADIANS_PER_MOTOR_ROTATION);
    leftEncoder.setVelocityConversionFactor(ARM_RADIANS_PER_MOTOR_ROTATION);
    absoluteEncoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);
    absoluteEncoder.setDistancePerRotation(2 * Math.PI);
  }

  /**
   * Sets the goal angle and enables periodic control.
   * 
   * @param goalAngle The goal angle in radians.
   */
  public void setGoalAngle(double goalAngle) {
    this.goalAngle = goalAngle;
    controller.reset(currentAngle);
    controller.setGoal(goalAngle);
    timer.reset();
    timer.start();
    enablePeriodicControl = true;
  }

  /**
   * Disables periodic control.
   */
  public void disable() {
    enablePeriodicControl = false;
    leftMotor.stopMotor();
    timer.stop();
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
    leftMotor.setVoltage(voltage);
  }

  /**
   * Sets the same power for both motors.
   * 
   * @param powerPercent The desired power. Range from -1.0 to 1.0.
   */
  public void setMotorPowers(double powerPercent) {
    powerPercent = MathUtil.clamp(powerPercent, -1.0, 1.0);
    leftMotor.set(powerPercent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentAngle = rawAngleOffset - absoluteEncoder.getDistance();

    if (enablePeriodicControl) {
      double feedback = controller.calculate(currentAngle);
      double feedForward = this.feedForward.calculate(currentAngle, controller.getSetpoint().velocity);
      setMotorVoltages(feedForward + feedback);
    }
  }

  public void addShuffleBoardTab() {
    if (!enableTab.getValue()) {
      return;
    }

    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
    ShuffleboardLayout infolayout = armTab.getLayout("Arm Info", BuiltInLayouts.kList)
      .withPosition(0, 0)
      .withSize(2, 4);
    infolayout.addDouble("Angle", () -> Math.toDegrees(getAngle()));
  }
}
