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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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
import frc.robot.parameters.ArmParameters;
import frc.robot.parameters.MotorParameters;
import java.util.Map;
import java.util.Set;

@RobotPreferencesLayout(
    groupName = "Arm+Shooter",
    column = 4,
    row = 0,
    width = 2,
    height = 4,
    type = "Grid Layout",
    gridColumns = 2,
    gridRows = 5)
public class ArmSubsystem extends SubsystemBase {

  @RobotPreferencesValue(column = 0, row = 0)
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Arm+Shooter", "Enable Tab", false);

  @RobotPreferencesValue(column = 0, row = 1)
  public static final RobotPreferences.DoubleValue KP =
      new RobotPreferences.DoubleValue("Arm+Shooter", "kP", 23.0);

  @RobotPreferencesValue(column = 0, row = 2)
  public static final RobotPreferences.DoubleValue KI =
      new RobotPreferences.DoubleValue("Arm+Shooter", "kI", 25.0);

  @RobotPreferencesValue(column = 0, row = 3)
  public static final RobotPreferences.DoubleValue KD =
      new RobotPreferences.DoubleValue("Arm+Shooter", "kD", 0.0);

  @RobotPreferencesValue(column = 0, row = 4)
  public static RobotPreferences.DoubleValue AMP_ANGLE =
      new RobotPreferences.DoubleValue("Arm+Shooter", "Amp Angle", 40.0);

  @RobotPreferencesValue(column = 1, row = 4)
  public static RobotPreferences.DoubleValue TRAP_ANGLE =
      new RobotPreferences.DoubleValue("Arm+Shooter", "Trap Angle", 70.0);

  @RobotPreferencesValue
  public static RobotPreferences.EnumValue<ArmParameters> PARAMETERS =
      new RobotPreferences.EnumValue<ArmParameters>(
          "Arm+Shooter", "Arm", ArmParameters.PracticeBase2024);

  public static final double GEAR_RATIO = PARAMETERS.getValue().getGearRatio();
  public static final double MASS = 12.5;
  public static final double RADIANS_PER_REVOLUTION = (2 * Math.PI) / GEAR_RATIO;
  public static final MotorParameters MOTOR = PARAMETERS.getValue().getMotorParameters();
  public static final double EFFICIENCY = 1.1;
  public static final double MAX_ANGULAR_SPEED =
      EFFICIENCY * MOTOR.getFreeSpeedRPM() * RADIANS_PER_REVOLUTION / 60.0;
  public static final double ARM_LENGTH = PARAMETERS.getValue().getArmLength();
  public static final double MAX_ANGULAR_ACCELERATION =
      EFFICIENCY * ((2 * MOTOR.getStallTorque() * GEAR_RATIO) / (MASS * ARM_LENGTH));
  public static final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * 0.3, MAX_ANGULAR_ACCELERATION * 0.5);
  public static final double KS = 0.15;
  public static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_SPEED;
  public static final double KA =
      (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_ACCELERATION;
  public static final double KG = KA * 9.81;
  private static final double CG_ANGLE_OFFSET =
      Math.toRadians(PARAMETERS.getValue().getCGAngleOffset());
  public static final double STOWED_ANGLE = Math.toRadians(PARAMETERS.getValue().getStowedAngle());
  private static final double NEARLY_STOWED_ANGLE = STOWED_ANGLE + Math.toRadians(3);
  private static final double ARM_RADIANS_PER_MOTOR_ROTATION = (2 * Math.PI) / GEAR_RATIO;
  private static final double LOWER_ANGLE_LIMIT = STOWED_ANGLE;
  private static final double UPPER_ANGLE_LIMIT = Math.toRadians(85);
  private static final double ANGLE_TOLERANCE = Math.toRadians(1);
  // private static final double LARGE_ANGLE_ERROR = Math.toRadians(20);

  private final CANSparkMax leftMotor =
      new CANSparkMax(RobotConstants.CAN.SparkMax.ARM_LEFT_PORT, MotorType.kBrushless);
  private final CANSparkMax rightMotor =
      new CANSparkMax(RobotConstants.CAN.SparkMax.ARM_RIGHT_PORT, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final DutyCycleEncoder absoluteEncoder =
      new DutyCycleEncoder(Constants.RobotConstants.DigitalIO.ARM_ABSOLUTE_ENCODER);

  private static final double rawAngleOffset =
      Math.toRadians(PARAMETERS.getValue().getRawAngleOffset());

  private double currentAngle = STOWED_ANGLE;
  private double currentGoal = STOWED_ANGLE;

  private double currentMotorVoltage = 0;
  private DoubleLogEntry currentAngleLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/Arm/Current Angle");
  private DoubleLogEntry currentGoalLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/Arm/Goal Angle");
  private DoubleLogEntry currentMotorVoltageLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/Arm/Motor Voltage");
  private DoubleLogEntry trapezoidStatePositionLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/Arm/State/Position");
  private DoubleLogEntry trapezoidStateVelocityLog =
      new DoubleLogEntry(DataLogManager.getLog(), "Arm/State/Velocity");
  private DoubleLogEntry trapezoidStatePositionErrorLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/Arm/State/Position Error");
  private DoubleLogEntry trapezoidStateVelocityErrorlog =
      new DoubleLogEntry(DataLogManager.getLog(), "/Arm/State/Velocity Error");
  private BooleanLogEntry enablePeriodicControlLog =
      new BooleanLogEntry(DataLogManager.getLog(), "Arm/Enabled");

  private final ArmFeedforward feedForward = new ArmFeedforward(KS, KG, KV, KA);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(KP.getValue(), KI.getValue(), KD.getValue(), CONSTRAINTS);
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

    // Limit the amount of integral windup
    controller.setIntegratorRange(-0.3, 0.3);
    controller.setIZone(Math.toRadians(3));

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
    controller.setPID(KP.getValue(), KI.getValue(), KD.getValue());
  }

  public void setGoalAngleContinous(double goalAngle) {
    this.currentGoal = goalAngle;
    controller.setGoal(goalAngle);
    enablePeriodicControl = true;
    enablePeriodicControlLog.append(enablePeriodicControl);
    currentGoalLog.append(goalAngle);
  }

  /** Disables periodic control. */
  public void disable() {
    enablePeriodicControl = false;
    enablePeriodicControlLog.append(enablePeriodicControl);
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
    currentMotorVoltageLog.append(voltage);
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
      currentAngleLog.append(currentAngle);
      // Turn off PID when nearly stowed
      if (currentAngle <= NEARLY_STOWED_ANGLE && currentGoal <= STOWED_ANGLE) {
        disable();
      } else {
        double feedback = controller.calculate(currentAngle);
        double cosine = Math.cos(currentAngle);
        feedback = MathUtil.clamp(feedback, 0.4 + -5.50 * cosine, -0.6 + 9.50 * cosine);
        // double error = controller.getPositionError();
        // System.out.println("error: " + Math.toDegrees(error));
        // if (Math.abs(error) < -LARGE_ANGLE_ERROR) {
        //   feedback = 0.4 - 5.50 * Math.cos(LARGE_ANGLE_ERROR);
        //   System.out.println("downward feedback: " + feedback);
        // }
        State setpoint = controller.getSetpoint();
        trapezoidStatePositionLog.append(setpoint.position);
        trapezoidStateVelocityLog.append(setpoint.velocity);
        trapezoidStatePositionErrorLog.append(controller.getPositionError());
        trapezoidStateVelocityErrorlog.append(controller.getVelocityError());
        double feedForward = 0;
        // this.feedForward.calculate(currentAngle + CG_ANGLE_OFFSET, setpoint.velocity);
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
    infolayout
        .addDouble(
            "Winch Position",
            () ->
                subsystems.climber.isPresent() ? subsystems.climber.get().getCurrentPosition() : 0)
        .withPosition(2, 3);

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
