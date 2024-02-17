// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.wpilibj2.command.Subsystem;

import com.nrg948.preferences.RobotPreferences;

/** Add your docs here. */
public class Subsystems {

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_POSE_ESTIMATION = new RobotPreferences.BooleanValue(
      "AprilTag", "Enable Pose Estimation", true);

  public final SwerveSubsystem drivetrain = new SwerveSubsystem();
  public final AprilTagSubsystem aprilTag = new AprilTagSubsystem();
  public final NoteVisionSubsystem noteVision = new NoteVisionSubsystem();
  public final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  public final AddressableLEDSubsystem addressableLEDSubsystem = new AddressableLEDSubsystem();
  public final ArmSubsystem armSubsystem = new ArmSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();

  public final Subsystem[] all = new Subsystem[] {
      drivetrain,
      aprilTag,
      noteVision,
      indexerSubsystem,
      intake,
      addressableLEDSubsystem,
      armSubsystem,
      shooter
  };

  public void periodic() {
    if (ENABLE_POSE_ESTIMATION.getValue()) {
      var visionEst = aprilTag.getEstimateGlobalPose();
      visionEst.ifPresent((est) -> {
        var estPose = est.estimatedPose.toPose2d();
        var estStdDevs = aprilTag.getEstimationStdDevs(estPose);

        drivetrain.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
      });
    }
  }
}
