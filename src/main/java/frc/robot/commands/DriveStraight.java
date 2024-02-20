/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

/** A command to drive the robot on a straight line in using trapezoidal motion profiling. */
public class DriveStraight extends Command {
  private final SwerveSubsystem drivetrain;
  private final HolonomicDriveController controller;
  private final Supplier<Translation2d> translationSupplier;
  private final double maxSpeed;
  private final Supplier<Rotation2d> orientationSupplier;
  private final Timer timer = new Timer();
  private Pose2d initialPose;
  private double distance;
  private Rotation2d heading;
  private Rotation2d orientation;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State initialState;
  private TrapezoidProfile.State goalState;

  /**
   * Creates a new DriveStraight that drives robot along the specified vector at maximum speed while
   * maintaining the current orientation of the robot.
   *
   * @param drivetrain The {@link SwerveSubsystem} representing the robot drivetrain.
   * @param translation A {@link Translation2d} instance describing the line on which to travel.
   *     This is a vector relative to the current position.
   */
  public DriveStraight(SwerveSubsystem drivetrain, Translation2d translation) {
    this(
        drivetrain,
        () -> translation,
        SwerveSubsystem.getMaxSpeed(),
        () -> drivetrain.getPosition().getRotation());
  }

  /**
   * Creates a new DriveStraight that drives robot along the specified vector and speed while
   * maintaining the current orientation of the robot.
   *
   * @param drivetrain The {@link SwerveSubsystem} representing the robot drivetrain.
   * @param translation A {@link Translation2d} instance describing the line on which to travel.
   *     This is a vector relative to the current position.
   * @param maxSpeed The maximum speed at which to travel.
   */
  public DriveStraight(SwerveSubsystem drivetrain, Translation2d translation, double maxSpeed) {
    this(drivetrain, () -> translation, maxSpeed, () -> drivetrain.getPosition().getRotation());
  }

  /**
   * Creates a new DriveStraight that drives robot along the specified vector while rotating the
   * robot to the desired orientation.
   *
   * @param drivetrain The {@link SwerveSubsystem} representing the robot drivetrain.
   * @param translation A {@link Translation2d} instance describing the line on which to travel.
   *     This is a vector relative to the current position.
   * @param maxSpeed The maximum speed at which to travel.
   * @param orientation The desired orientation at the end of the command.
   */
  public DriveStraight(
      SwerveSubsystem drivetrain,
      Translation2d translation,
      double maxSpeed,
      Rotation2d orientation) {
    this(drivetrain, () -> translation, maxSpeed, () -> orientation);
  }

  /**
   * Creates a new DriveStraight that drives robot to the specified absolute location and
   * orientation on the field.
   *
   * @param drivetrain The {@link SwerveSubsystem} representing the robot drivetrain.
   * @param position A {@link Pose2d} instance describing the absolute position and orientation to
   *     drive to.
   * @param maxSpeed The maximum speed at which to travel.
   */
  public DriveStraight(SwerveSubsystem drivetrain, Pose2d position, double maxSpeed) {
    this(
        drivetrain,
        () -> position.getTranslation().minus(drivetrain.getPosition().getTranslation()),
        maxSpeed,
        () -> position.getRotation());
  }

  /**
   * Constructs an instance of this class.
   *
   * @param drivetrain The {@link SwerveSubsystem} representing the robot drivetrain.
   * @param translationSupplier Supplies a {@link Translation2d} instance describing the line on
   *     which to travel. This is a vector relative to the current position.
   * @param maxSpeed The maximum speed at which to travel.
   * @param orientationSupplier Supplies the desired orientation at the end of the command.
   */
  private DriveStraight(
      SwerveSubsystem drivetrain,
      Supplier<Translation2d> translationSupplier,
      double maxSpeed,
      Supplier<Rotation2d> orientationSupplier) {
    this.drivetrain = drivetrain;
    this.translationSupplier = translationSupplier;
    this.controller = drivetrain.createDriveController();
    this.maxSpeed = maxSpeed;
    this.orientationSupplier = orientationSupplier;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    initialPose = drivetrain.getPosition();

    Translation2d translation = translationSupplier.get();
    distance = translation.getNorm();
    heading = translation.getAngle();
    orientation = orientationSupplier.get();
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxSpeed, SwerveSubsystem.getMaxAcceleration()));

    initialState = new TrapezoidProfile.State(0, 0);
    goalState = new TrapezoidProfile.State(distance, 0);

    System.out.println(
        "BEGIN DriveStraight intitialPose = "
            + initialPose
            + ", orientation = "
            + orientation
            + ", distance = "
            + distance
            + ", heading = "
            + heading);

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    // Calculate the next state (position and velocity) of motion using the
    // trapezoidal profile.
    TrapezoidProfile.State state = profile.calculate(timer.get(), initialState, goalState);

    // Determine the next position on the field by offsetting the initial position
    // by the distance moved along the line of travel.
    Translation2d offset = new Translation2d(state.position, heading);
    Pose2d nextPose = new Pose2d(initialPose.getTranslation().plus(offset), heading);

    // Calculate the swerve drive modules states needed to reach the next state.
    ChassisSpeeds speeds =
        controller.calculate(drivetrain.getPosition(), nextPose, state.velocity, orientation);

    drivetrain.setChassisSpeeds(speeds);
  }

  @Override
  public boolean isFinished() {
    return profile.isFinished(timer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
    timer.stop();
    System.out.println("END DriveStraight finalPose = " + drivetrain.getPosition());
  }
}
