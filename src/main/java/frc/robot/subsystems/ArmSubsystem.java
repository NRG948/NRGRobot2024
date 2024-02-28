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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.ArmCommands;
import frc.robot.parameters.MotorParameters;
import java.util.Map;
import java.util.Set;

@RobotPreferencesLayout(groupName = "Arm+Shooter", column = 5, row = 0, width = 1, height = 3)
public class ArmSubsystem extends SubsystemBase {

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Arm+Shooter", "Enable Tab", false);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue KP =
      new RobotPreferences.DoubleValue("Arm+Shooter", "kP", 10.0);

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue AMP_ANGLE =
      new RobotPreferences.DoubleValue("Arm+Shooter", "Amp Angle", 8.0);

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue TRAP_ANGLE =
      new RobotPreferences.DoubleValue("Arm+Shooter", "Trap Angle", 45);

  public static final double GEAR_RATIO = (5 * 4 * 3 * 42 / 15.0);
  public static final double MASS = 12.5;
  public static final double RADIANS_PER_REVOLUTION = (2 * Math.PI) / GEAR_RATIO;
  public static final MotorParameters MOTOR = MotorParameters.NeoV1_1;
  public static final double EFFICIENCY = 1;
  public static final double MAX_ANGULAR_SPEED =
      EFFICIENCY * MOTOR.getFreeSpeedRPM() * RADIANS_PER_REVOLUTION / 60.0;
  public static final double ARM_LENGTH = 0.57;
  public static final double MAX_ANGULAR_ACCELERATION =
      EFFICIENCY * ((2 * MOTOR.getStallTorque() * GEAR_RATIO) / (MASS * ARM_LENGTH));
  public static final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * 0.3, MAX_ANGULAR_ACCELERATION * 0.5);
  public static final double KS = 0.15;
  public static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_SPEED;
  public static final double KA =
      (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_ACCELERATION;
  public static final double KG = KA * 9.81;
  private static final double CG_ANGLE_OFFSET = Math.toRadians(8);
  public static final double STOWED_ANGLE = Math.toRadians(-11.0);
  private static final double NEARLY_STOWED_ANGLE = STOWED_ANGLE + Math.toRadians(1.5);
  private static final double ARM_RADIANS_PER_MOTOR_ROTATION = (2 * Math.PI) / GEAR_RATIO;
  private static final double LOWER_ANGLE_LIMIT = STOWED_ANGLE;
  private static final double UPPER_ANGLE_LIMIT = Math.toRadians(70);
  private static final double ANGLE_TOLERANCE = Math.toRadians(3);

  private final CANSparkMax leftMotor =
      new CANSparkMax(RobotConstants.CAN.SparkMax.ARM_LEFT_PORT, MotorType.kBrushless);
  private final CANSparkMax rightMotor =
      new CANSparkMax(RobotConstants.CAN.SparkMax.ARM_RIGHT_PORT, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final DutyCycleEncoder absoluteEncoder =
      new DutyCycleEncoder(Constants.RobotConstants.DigitalIO.ARM_ABSOLUTE_ENCODER);

  private double rawAngleOffset = Math.toRadians(7.39);

  private double currentAngle = STOWED_ANGLE;
  private double currentGoal = STOWED_ANGLE;

  private double currentMotorVoltage = 0;

  private final ArmFeedforward feedForward = new ArmFeedforward(KS, KG, KV, KA);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(KP.getValue(), 0.0, 0.0, CONSTRAINTS);
  private boolean enablePeriodicControl = false;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    rightMotor.follow(leftMotor, true);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftEncoder.setPositionConversionFactor(ARM_RADIANS_PER_MOTOR_ROTATION);
    leftEncoder.setVelocityConversionFactor(ARM_RADIANS_PER_MOTOR_ROTATION);
    absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    absoluteEncoder.setDistancePerRotation(2 * Math.PI);
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(ANGLE_TOLERANCE);

    System.out.println("Arm max velocity: " + Math.toDegrees(MAX_ANGULAR_SPEED));
    System.out.println("Arm max accel: " + Math.toDegrees(MAX_ANGULAR_ACCELERATION));
  }

  /**
   * Sets the goal angle and enables periodic control.
   *
   * @param goalAngle The goal angle in radians.
   */
  public void setGoalAngle(double goalAngle) {
    setGoalAngleContinous(goalAngle);
    controller.reset(currentAngle);
    controller.setP(KP.getValue());
  }

  public void setGoalAngleContinous(double goalAngle) {
    this.currentGoal = goalAngle;
    controller.setGoal(goalAngle);
    enablePeriodicControl = true;
  }

  /** Disables periodic control. */
  public void disable() {
    enablePeriodicControl = false;
    leftMotor.stopMotor();
    currentMotorVoltage = 0;
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
    currentMotorVoltage = voltage;
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
      // Turn off PID when nearly stowed
      if (currentAngle <= NEARLY_STOWED_ANGLE && currentGoal <= STOWED_ANGLE) {
        disable();
      } else {
        double feedback = controller.calculate(currentAngle);
        double feedForward =
            this.feedForward.calculate(
                currentAngle + CG_ANGLE_OFFSET, controller.getSetpoint().velocity);
        setMotorVoltages(feedForward + feedback);
      }
    }
  }

  public void addShuffleboardLayout(ShuffleboardTab tab, Subsystems subsystems) {
    ShuffleboardLayout infolayout =
        tab.getLayout("Arm Info", BuiltInLayouts.kGrid)
            .withPosition(0, 0)
            .withSize(2, 4)
            .withProperties(Map.of("Number of columns", 3, "Number of rows", 4));
    infolayout
        .addDouble("Current Angle", () -> Math.toDegrees(getAngle()))
        .withPosition(0, 0)
        .withSize(1, 1);
    infolayout
        .addDouble("Goal Angle", () -> Math.toDegrees(controller.getGoal().position))
        .withPosition(1, 0)
        .withSize(1, 1);
    infolayout.addDouble("Voltage", () -> currentMotorVoltage).withPosition(2, 0).withSize(1, 1);
    infolayout.add("Max Velocity", MAX_ANGULAR_SPEED).withPosition(0, 1).withSize(1, 1);
    infolayout.add("Max Accel", MAX_ANGULAR_ACCELERATION).withPosition(1, 1).withSize(1, 1);
    infolayout.addBoolean("Enabled", () -> enablePeriodicControl).withPosition(2, 1).withSize(1, 1);
    infolayout.add("KV", KV).withPosition(0, 2).withSize(1, 1);
    infolayout.add("KA", KA).withPosition(1, 2).withSize(1, 1);
    infolayout.add("KG", KG).withPosition(2, 2).withSize(1, 1);

    ShuffleboardLayout controlLayout =
        tab.getLayout("Arm Control", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);
    GenericEntry armAngle = controlLayout.add("Arm Angle", 0).getEntry();
    controlLayout.add(
        Commands.defer(
            () ->
                ArmCommands.seekToAngle(subsystems, Math.toRadians(armAngle.getDouble(ARM_LENGTH)))
                    .until(() -> this.atGoalAngle()),
            Set.of(subsystems.arm)));
  }
}
