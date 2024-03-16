/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

/** A container class to manage all of the robot subsystems. */
public class Subsystems {

  @RobotPreferencesValue(column = 0, row = 1)
  public static final RobotPreferences.BooleanValue ENABLE_POSE_ESTIMATION =
      new RobotPreferences.BooleanValue("AprilTag", "Enable Pose Estimation", true);

  public final SwerveSubsystem drivetrain = new SwerveSubsystem();
  public final IndexerSubsystem indexer = new IndexerSubsystem();
  public final StatusLEDSubsystem statusLED = new StatusLEDSubsystem();
  public final ArmSubsystem arm = new ArmSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem(() -> drivetrain.getOrientation());

  public final Optional<AprilTagSubsystem> aprilTag;
  public final Optional<NoteVisionSubsystem> noteVision;
  public final Optional<ClimberSubsystem> climber;

  public final Subsystem[] all;

  /** Initializes all of the robot subsystems. */
  public Subsystems() {
    ArrayList<Subsystem> all =
        new ArrayList<Subsystem>(
            Arrays.asList(drivetrain, indexer, intake, statusLED, arm, shooter));

    aprilTag = newOptionalSubsystem(AprilTagSubsystem.class, AprilTagSubsystem.ENABLED);
    if (aprilTag.isPresent()) {
      all.add(aprilTag.get());
    }

    noteVision = newOptionalSubsystem(NoteVisionSubsystem.class, NoteVisionSubsystem.ENABLED);
    if (noteVision.isPresent()) {
      all.add(noteVision.get());
    }

    climber = newOptionalSubsystem(ClimberSubsystem.class, ClimberSubsystem.ENABLED);
    if (climber.isPresent()) {
      all.add(climber.get());
    }

    this.all = all.toArray(Subsystem[]::new);
  }

  /**
   * Creates a new optional subsystem.
   *
   * @param <T> The type of subsystem.
   * @param subsystemClass The subsystem class.
   * @param enabled The preferences value indicating whether the subsystem is enabled.
   * @return Returns a non-empty {@link Optional} instance if the subsystem is enabled. Otherwise,
   *     this method returns {@link Optional#empty}.
   */
  private static <T extends Subsystem> Optional<T> newOptionalSubsystem(
      Class<T> subsystemClass, RobotPreferences.BooleanValue enabled) {
    if (!enabled.getValue()) {
      return Optional.empty();
    }

    try {
      return Optional.of(subsystemClass.getConstructor().newInstance());
    } catch (InstantiationException
        | IllegalAccessException
        | IllegalArgumentException
        | InvocationTargetException
        | NoSuchMethodException
        | SecurityException e) {
      System.err.printf(
          "ERROR: An unexpected exception was caught while creating an instance of %s.%n",
          subsystemClass.getName());
      e.printStackTrace();
      return Optional.empty();
    }
  }

  public void periodic() {
    if (aprilTag.isPresent() && ENABLE_POSE_ESTIMATION.getValue()) {
      AprilTagSubsystem aprilTag = this.aprilTag.get();
      var visionEst = aprilTag.getEstimateGlobalPose();

      visionEst.ifPresent(
          (est) -> {
            var estPose = est.estimatedPose.toPose2d();
            var estStdDevs = aprilTag.getEstimationStdDevs(estPose);

            drivetrain.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
          });
    }
  }
}
