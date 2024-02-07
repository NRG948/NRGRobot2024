// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferencesValue;
import com.nrg948.preferences.RobotPreferences;

/** Add your docs here. */
public class Subsystems {
  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_POSE_ESTIMATION = new RobotPreferences.BooleanValue(
      "AprilTag", "Enable Pose Estimation", true);
      
  public SwerveSubsystem drivetrain = new SwerveSubsystem();
  public AprilTagSubsystem aprilTag = new AprilTagSubsystem();
  public NoteVisionSubsystem noteVision = new NoteVisionSubsystem();
  public IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  public AddressableLEDSubsystem addressableLEDSubsystem = new AddressableLEDSubsystem();
  public ArmSubsystem armSubsystem = new ArmSubsystem();
  // public IntakeSubsystem intake = new IntakeSubsystem();

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
