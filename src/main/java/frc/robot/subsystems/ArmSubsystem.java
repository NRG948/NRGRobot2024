// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.parameters.MotorParameters;

public class ArmSubsystem extends SubsystemBase {
  public static final double GEAR_RATIO = 75.0; // TODO determine actual gear ratio
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
  private final RelativeEncoder encoder = leftMotor.getEncoder();
  private double currentAngle = STOWED_ANGLE;
  private final ArmFeedforward feedForward = new ArmFeedforward(KS, KV, KA, KG);
  private final Timer timer = new Timer();
  private final ProfiledPIDController controller = new ProfiledPIDController(1.0, 0.0, 0.0, CONSTRAINTS);
  private double goalAngle;
  private boolean enablePeriodicControl = false;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    rightMotor.setInverted(true); // TODO verify correct inversion
    rightMotor.follow(leftMotor);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    encoder.setPositionConversionFactor(ARM_RADIANS_PER_MOTOR_ROTATION);
    encoder.setVelocityConversionFactor(ARM_RADIANS_PER_MOTOR_ROTATION);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentAngle = encoder.getPosition();

    if (enablePeriodicControl) {
      double feedback = controller.calculate(currentAngle);
      double feedForward = this.feedForward.calculate(currentAngle, controller.getSetpoint().velocity);
      leftMotor.setVoltage(feedForward + feedback);
    }

  }
}
