/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.parameters;

import static frc.robot.parameters.MotorParameters.Falcon500;
import static frc.robot.parameters.MotorParameters.NeoV1_1;
import static frc.robot.parameters.SwerveModuleParameters.MK4IFast;
import static frc.robot.parameters.SwerveModuleParameters.MK4Standard;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;

/** An enum representing the properties for the swerve drive base of a specific robot instance. */
public enum SwerveDriveParameters {

  /**
   * The 2022 competition robot using feedforward constants calculated from theoretical maximums.
   */
  RookieBase2023(
      67.5853,
      Units.inchesToMeters(22.729),
      Units.inchesToMeters(19.39),
      MK4Standard,
      Falcon500,
      NeoV1_1,
      new int[] {2, 3, 4, 5, 6, 7, 8, 9}, // drive, steer motor controller CAN IDs
      new int[] {12, 14, 16, 18}, // CANCoder CAN IDs
      new double[] {144.23, 26.02, 201.18, 202.59},
      0.15,
      0.15),
  RookieBase2023Characterized(
      67.5853,
      Units.inchesToMeters(22.729),
      Units.inchesToMeters(19.39),
      MK4Standard,
      Falcon500,
      NeoV1_1,
      new int[] {2, 3, 4, 5, 6, 7, 8, 9}, // drive, steer motor controller CAN IDs
      new int[] {12, 14, 16, 18}, // CANCoder CAN IDs
      new double[] {144.23, 26.02, 201.18, 202.59},
      new FeedforwardConstants(0.13638, 2.3316, 0.058878),
      new FeedforwardConstants(0.13638, 2.3316, 0.058878) // TODO; characterize steering
      ),
  PracticeBase2024(
      24.2,
      Units.inchesToMeters(22.55),
      Units.inchesToMeters(22.55),
      MK4IFast,
      Falcon500,
      NeoV1_1,
      new int[] {6, 7, 19, 18, 8, 9, 13, 14}, // drive, steer motor controller CAN IDs
      new int[] {31, 32, 33, 34}, // CANCoder CAN IDs
      new double[] {186.5, 69.61, 172.62, 21.27},
      0.15,
      0.15),
  PracticeBase2024Characterized(
      24.2,
      Units.inchesToMeters(22.55),
      Units.inchesToMeters(22.55),
      MK4IFast,
      Falcon500,
      NeoV1_1,
      new int[] {6, 7, 19, 18, 8, 9, 13, 14}, // drive, steer motor controller CAN IDs
      new int[] {31, 32, 33, 34}, // CANCoder CAN IDs
      new double[] {186.5, 69.61, 172.62, 21.27},
      new FeedforwardConstants(0.14566, 2.3072, 0.0393),
      new FeedforwardConstants(0.11411975, 157.9575, 10.1638)), // TODO: Redo characterization
  CompetitionBase2024(
      24.2, // TODO: Find out real mass
      Units.inchesToMeters(22.55),
      Units.inchesToMeters(22.55),
      MK4IFast,
      Falcon500,
      NeoV1_1,
      new int[] {6, 7, 19, 18, 8, 9, 13, 14}, // drive, steer motor controller CAN IDs
      new int[] {31, 32, 33, 34}, // CANCoder CAN IDs
      new double[] {169.10, 141.77, 120.41, 201.27}, // TODO: Update Offsets
      0.15,
      0.15);

  public static class Constants {
    /**
     * A scaling factor used to adjust from theoretical maximums given that any physical system
     * generally cannot achieve them.
     */
    public static final double SCALE_FACTOR = 0.8;
  }

  private final double robotMass;
  private final double wheelDistanceX;
  private final double wheelDistanceY;
  private final SwerveModuleParameters swerveModule;
  private final MotorParameters driveMotor;
  private final MotorParameters steeringMotor;
  private final int[] motorIds;
  private final int[] angleEncoderIds;
  private final double[] angleOffset;

  private final double maxDriveSpeed;
  private final double maxDriveAcceleration;

  private final FeedforwardConstants driveFeedforward;

  private final double maxSteeringSpeed;
  private final double maxSteeringAcceleration;

  private final FeedforwardConstants steeringFeedforward;

  private final double maxRotationalSpeed;
  private final double maxRotationalAcceleration;

  private final Translation2d[] wheelPositions;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveKinematicsConstraint kinematicsConstraint;
  private final TrapezoidProfile.Constraints steeringConstraints;
  private final TrapezoidProfile.Constraints robotRotationalConstraints;

  /**
   * Constructs an instance of this enum.
   *
   * <p><b>NOTE:</b> The distance between wheels are expressed in the NWU coordinate system relative
   * to the robot frame as shown below.
   *
   * <p>
   *
   * <pre>
   * <code>
   *            ^
   * +--------+ | x
   * |O      O| |
   * |        | |
   * |        | |
   * |O      O| |
   * +--------+ |
   *            |
   * <----------+
   *  y       (0,0)
   * </code>
   * </pre>
   *
   * @param robotMass The mass of the robot in Kg.
   * @param wheelDistanceX The distance between the wheels along the X axis in meters.
   * @param wheelDistanceY The distance between the wheels along the Y axis in meters.
   * @param swerveModule The swerve module used by the robot.
   * @param driveMotor The motor used by swerve module drive on the robot.
   * @param steeringMotor The motor used by the swerve module steering on the robot.
   * @param motorIds An array containing the CAN IDs of the swerve module drive motors in the order
   *     front left drive and steering, front right drive and steering, back left drive and
   *     steering, back right drive and steering.
   * @param angleEncoderIds An array containing the CAN IDs of the swerve module angle encoders in
   *     the order front left, front right, back left, back right.
   * @param driveFeedforward The drive feedforward constants.
   * @param steeringFeedforward The steering feedforward constants.
   */
  private SwerveDriveParameters(
      double robotMass,
      double wheelDistanceX,
      double wheelDistanceY,
      SwerveModuleParameters swerveModule,
      MotorParameters driveMotor,
      MotorParameters steeringMotor,
      int[] motorIds,
      int[] angleEncoderIds,
      double[] angleOffset,
      FeedforwardConstants driveFeedForward,
      FeedforwardConstants steeringFeedForward) {
    this.robotMass = robotMass;
    this.wheelDistanceX = wheelDistanceX;
    this.wheelDistanceY = wheelDistanceY;
    this.swerveModule = swerveModule;
    this.driveMotor = driveMotor;
    this.steeringMotor = steeringMotor;
    this.motorIds = motorIds;
    this.angleEncoderIds = angleEncoderIds;
    this.angleOffset = angleOffset;
    this.driveFeedforward = driveFeedForward;
    this.steeringFeedforward = steeringFeedForward;

    double scaleFactor = Constants.SCALE_FACTOR;

    this.maxDriveSpeed = scaleFactor * this.swerveModule.calculateMaxDriveSpeed(this.driveMotor);
    this.maxDriveAcceleration =
        scaleFactor
            * this.swerveModule.calculateMaxDriveAcceleration(this.driveMotor, this.robotMass);

    this.maxSteeringSpeed =
        scaleFactor * this.swerveModule.calculateMaxSteeringSpeed(this.steeringMotor);
    this.maxSteeringAcceleration =
        scaleFactor
            * this.swerveModule.calculateMaxSteeringAcceleration(
                this.steeringMotor, this.robotMass);

    final double wheelTrackRadius = Math.hypot(this.wheelDistanceX, this.wheelDistanceY);

    this.maxRotationalSpeed = this.maxDriveSpeed / wheelTrackRadius;
    this.maxRotationalAcceleration = this.maxDriveAcceleration / wheelTrackRadius;

    this.wheelPositions =
        new Translation2d[] {
          new Translation2d(this.wheelDistanceX / 2.0, this.wheelDistanceY / 2.0),
          new Translation2d(this.wheelDistanceX / 2.0, -this.wheelDistanceY / 2.0),
          new Translation2d(-this.wheelDistanceX / 2.0, this.wheelDistanceY / 2.0),
          new Translation2d(-this.wheelDistanceX / 2.0, -this.wheelDistanceY / 2.0),
        };

    this.kinematics = new SwerveDriveKinematics(wheelPositions);
    this.kinematicsConstraint = new SwerveDriveKinematicsConstraint(kinematics, this.maxDriveSpeed);
    this.steeringConstraints =
        new TrapezoidProfile.Constraints(this.maxSteeringSpeed, this.maxSteeringAcceleration);
    this.robotRotationalConstraints =
        new TrapezoidProfile.Constraints(this.maxRotationalSpeed, this.maxRotationalAcceleration);
  }

  /**
   * Constructs an instance of this enum.
   *
   * <p><b>NOTE:</b> The distance between wheels are expressed in the NWU coordinate system relative
   * to the robot frame as shown below.
   *
   * <p>
   *
   * <pre>
   * <code>
   *            ^
   * +--------+ | x
   * |O      O| |
   * |        | |
   * |        | |
   * |O      O| |
   * +--------+ |
   *            |
   * <----------+
   *  y       (0,0)
   * </code>
   * </pre>
   *
   * @param robotMass The mass of the robot in Kg.
   * @param wheelDistanceX The distance between the wheels along the X axis in meters.
   * @param wheelDistanceY The distance between the wheels along the Y axis in meters
   * @param swerveModule The swerve module used by the robot.
   * @param driveMotor The motor used by swerve module drive on the robot.
   * @param steeringMotor The motor used by the swerve module steering on the robot.
   * @param motorIds An array containing the CAN IDs of the swerve module drive motors in the order
   *     front left drive and steering, front right drive and steering, back left drive and
   *     steering, back right drive and steering.
   * @param angleEncoderIds An array containing the CAN IDs of the swerve module angle encoders in
   *     the order front left, front right, back left, back right.
   * @param driveKs The drive kS constant.
   * @param steeringKs The steering kS constant.
   */
  private SwerveDriveParameters(
      double robotMass,
      double wheelDistanceX,
      double wheelDistanceY,
      SwerveModuleParameters swerveModule,
      MotorParameters driveMotor,
      MotorParameters steeringMotor,
      int[] motorIds,
      int[] angleEncoderIds,
      double[] angleOffset,
      double drivekS,
      double steeringkS) {
    this(
        robotMass,
        wheelDistanceX,
        wheelDistanceY,
        swerveModule,
        driveMotor,
        steeringMotor,
        motorIds,
        angleEncoderIds,
        angleOffset,
        new CalculatedFeedforwardConstants(
            drivekS,
            () -> swerveModule.calculateMaxDriveSpeed(driveMotor),
            () -> swerveModule.calculateMaxDriveAcceleration(driveMotor, robotMass)),
        new CalculatedFeedforwardConstants(
            steeringkS,
            () -> swerveModule.calculateMaxSteeringSpeed(steeringMotor),
            () -> swerveModule.calculateMaxSteeringAcceleration(steeringMotor, robotMass)));
  }

  /**
   * Returns the mass of the robot in Kg.
   *
   * @return The mass of the robot in Kg.
   */
  public double getRobotMass() {
    return this.robotMass;
  }

  /**
   * Returns the maximum rotational speed of the robot in rad/s.
   *
   * @return The maximum rotation speed of the robot in rad/s.
   */
  public double getMaxRotationalSpeed() {
    return this.maxRotationalSpeed;
  }

  /**
   * Returns the maximum rotational acceleration of the robot in rad/s^2.
   *
   * @return The maximum rotation acceleration of the robot in rad/s^s.
   */
  public double getMaxRotationalAcceleration() {
    return this.maxRotationalAcceleration;
  }

  /**
   * Return the wheel base radius in meters.
   *
   * @return The wheel base radius in meters.
   */
  public double getWheelBaseRadius() {
    double halfWheelDistanceX = wheelDistanceX / 2;
    double halfWheelDistanceY = wheelDistanceY / 2;

    return Math.sqrt(
        halfWheelDistanceX * halfWheelDistanceX + halfWheelDistanceY * halfWheelDistanceY);
  }

  /**
   * Returns a {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   * constraints on the {@link ProfilePIDController} used to reach the goal robot orientation.
   *
   * @return A {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   *     constraints on the controller used to reach the goal robot orientation.
   */
  public TrapezoidProfile.Constraints getRotationalConstraints() {
    return this.robotRotationalConstraints;
  }

  /**
   * Returns the distance between the wheels along the X-axis in meters.
   *
   * @return The distance between the wheels along the X-axis in meters.
   */
  public double getWheelDistanceX() {
    return this.wheelDistanceX;
  }

  /**
   * Returns the distance between the wheels along the Y-axis in meters.
   *
   * @return The distance between the wheels along the Y-axis in meters.
   */
  public double getWheelDistanceY() {
    return this.wheelDistanceY;
  }

  /**
   * Returns the wheel positions relative to the robot center.
   *
   * @return The wheel positions relative to the robot center.
   */
  public Translation2d[] getWheelPositions() {
    return this.wheelPositions;
  }

  /**
   * Returns the wheel diameter in meters.
   *
   * @return The wheel diameter in meters.
   */
  public double getWheelDiameter() {
    return this.swerveModule.getWheelDiameter();
  }

  /**
   * Returns the drive gear ratio.
   *
   * @return The drive gear ratio.
   */
  public double getDriveGearRatio() {
    return this.swerveModule.getDriveGearRation();
  }

  /**
   * Returns the swerve module used by the robot.
   *
   * @return The swerve module used by the robot.
   */
  public SwerveModuleParameters getSwerveModule() {
    return this.swerveModule;
  }

  /**
   * Returns the motor used by swerve module on the robot.
   *
   * @return The motor used by swerve module on the robot.
   */
  public MotorParameters getMotorParameters() {
    return this.driveMotor;
  }

  /**
   * Returns the CAN ids of the specified motor.
   *
   * @param motor The motor.
   * @return The CAN id.
   */
  public int getMotorId(SwerveMotors motor) {
    return this.motorIds[motor.getIndex()];
  }

  /**
   * Returns the CANcoder ids of the specified module.
   *
   * @param angleEncoder The angle encoder.
   * @return The CANcoder id.
   */
  public int getAngleEncoderId(SwerveAngleEncoder angleEncoder) {
    return this.angleEncoderIds[angleEncoder.getIndex()];
  }

  /**
   * Returns the angle offsets for the specified module.
   *
   * @return the angle offsets for the specified module.
   */
  public double getAngleOffset(SwerveAngleEncoder angleEncoder) {
    return this.angleOffset[angleEncoder.getIndex()];
  }

  /**
   * Returns the maximum drive speed in m/s of a swerve module.
   *
   * @return The maximum drive speed.
   */
  public double getMaxDriveSpeed() {
    return this.maxDriveSpeed;
  }

  /**
   * Returns the maximum drive acceleration in m/s^2 of a swerve module.
   *
   * @return The maximum drive acceleration.
   */
  public double getMaxDriveAcceleration() {
    return this.maxDriveAcceleration;
  }

  /**
   * Returns the kS feedforward control constant for translation in Volts.
   *
   * <p>This is the voltage needed to overcome the internal friction of the motor.
   *
   * @return The kS feedforward control constant for translation in Volts.
   */
  public double getDriveKs() {
    return this.driveFeedforward.kS;
  }

  /**
   * Returns the kV feedforward control constant for translation in Volt * seconds per meter.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant velocity.
   *
   * @return The kV feedforward control constant for translation in Volt * seconds per meter.
   */
  public double getDriveKv() {
    return this.driveFeedforward.kV;
  }

  /**
   * Returns the kA feedforward control constant for translation in Volt * seconds^2 per meter.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant acceleration.
   *
   * @return The kA feedforward control constant for translation in Volt * seconds^2 per meter.
   */
  public double getDriveKa() {
    return this.driveFeedforward.kA;
  }

  /**
   * Returns the pulses per meter of the integrated encoder.
   *
   * @return The pulses per meter of the integrated encoder.
   */
  public double getDrivePulsesPerMeter() {
    return this.swerveModule.calculateDrivePulsesPerMeter(this.driveMotor);
  }

  /**
   * Returns a {@link SwerveDriveKinematics} object used to convert chassis speeds to individual
   * module states.
   *
   * @return A {@link SwerveDriveKinematics} object used to convert chassis speeds to individual
   *     module states.
   */
  public SwerveDriveKinematics getKinematics() {
    return this.kinematics;
  }

  /**
   * Returns a {@link SwerveDriveKinematicsConstraint} object used to enforce swerve drive
   * kinematics constraints when following a trajectory.
   *
   * @return A {@link SwerveDriveKinematicsConstraint} object used to enforce swerve drive
   *     kinematics constraints when following a trajectory.
   */
  public SwerveDriveKinematicsConstraint getKinematicsConstraint() {
    return this.kinematicsConstraint;
  }

  /**
   * Returns the maximum steering speed in rad/s of a swerve module.
   *
   * @return The maximum steering speed.
   */
  public double getMaxSteeringSpeed() {
    return this.maxSteeringSpeed;
  }

  /**
   * Returns the maximum steering acceleration in rad/s^2 of a swerve module.
   *
   * @return The maximum steering acceleration.
   */
  public double getMaxSteeringAcceleration() {
    return this.maxSteeringAcceleration;
  }

  /**
   * Returns the kS feedforward control constant for rotation in Volts.
   *
   * <p>This is the voltage needed to overcome the internal friction of the motor.
   *
   * @return The kS feedforward control constant for rotation in Volts.
   */
  public double getSteeringKs() {
    return this.steeringFeedforward.kS;
  }

  /**
   * Returns the kV feedforward control constant for rotation in Volt * seconds per radian.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant steering velocity.
   *
   * @return The kV feedforward control constant for translation in Volt * seconds per radian.
   */
  public double getSteeringKv() {
    return this.steeringFeedforward.kV;
  }

  /**
   * Returns the kA feedforward control constant for rotation in Volt * seconds^2 per radian.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant rotational acceleration.
   *
   * @return The kA feedforward control constant for translation in Volt * seconds^2 per radian.
   */
  public double getSteeringKa() {
    return this.steeringFeedforward.kA;
  }

  /**
   * Returns a {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   * constraints on the {@link ProfiledPIDController} used to reach the goal wheel angle.
   *
   * @return A {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   *     constraints on the controller used to reach the goal wheel angle.
   */
  public TrapezoidProfile.Constraints getSteeringConstraints() {
    return steeringConstraints;
  }

  /**
   * Returns true if the steering motor is inverted.
   *
   * @return Whether the steering motor is inverted.
   */
  public boolean isSteeringInverted() {
    return swerveModule.isSteeringInverted();
  }
}
