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
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

/** Add your docs here. */
public class Subsystems {

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_POSE_ESTIMATION =
      new RobotPreferences.BooleanValue("AprilTag", "Enable Pose Estimation", true);

  public final SwerveSubsystem drivetrain = new SwerveSubsystem();
  public final Optional<AprilTagSubsystem> aprilTag;
  public final Optional<NoteVisionSubsystem> noteVision;
  public final IndexerSubsystem indexer = new IndexerSubsystem();
  public final StatusLEDSubsystem statusLED = new StatusLEDSubsystem();
  public final ArmSubsystem arm = new ArmSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();

  public final Subsystem[] all;

  public Subsystems() {
    ArrayList<Subsystem> all =
        new ArrayList<Subsystem>(
            Arrays.asList(drivetrain, indexer, intake, statusLED, arm, shooter));

    if (AprilTagSubsystem.ENABLED.getValue()) {
      aprilTag = Optional.of(new AprilTagSubsystem());
      all.add(aprilTag.get());
    } else {
      aprilTag = Optional.empty();
    }

    if (NoteVisionSubsystem.ENABLED.getValue()) {
      noteVision = Optional.of(new NoteVisionSubsystem());
      all.add(noteVision.get());
    } else {
      noteVision = Optional.empty();
    }

    this.all = all.toArray(Subsystem[]::new);
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
