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

/** Add your docs here. */
public class Subsystems {

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_POSE_ESTIMATION =
      new RobotPreferences.BooleanValue("AprilTag", "Enable Pose Estimation", true);

  public final SwerveSubsystem drivetrain = new SwerveSubsystem();
  public final AprilTagSubsystem aprilTag = new AprilTagSubsystem();
  public final NoteVisionSubsystem noteVision = new NoteVisionSubsystem();
  public final IndexerSubsystem indexer = new IndexerSubsystem();
  public final AddressableLEDSubsystem addressableLED = new AddressableLEDSubsystem();
  public final ArmSubsystem arm = new ArmSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();

  public final Subsystem[] all =
      new Subsystem[] {
        drivetrain, aprilTag, noteVision, indexer, intake, addressableLED, arm, shooter
      };

  public void periodic() {
    if (ENABLE_POSE_ESTIMATION.getValue()) {
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
