/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.NoteVisionSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystemBase;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Map;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public final class DriveCommands {

  public static Rotation2d ROTATE_180_DEGREES = Rotation2d.fromDegrees(180);

  @RobotPreferencesValue(column = 1, row = 1)
  public static final RobotPreferences.BooleanValue USE_ESTIMATED_POSE =
      new RobotPreferences.BooleanValue("AprilTag", "Use Estimated Pose", false);

  /** Returns a Command that drives to amp delivery position. */
  public static Command driveToAmp(Subsystems subsystems) {

    if (subsystems.aprilTag.isEmpty()) {
      return Commands.none(); // TODO blink LEDS red to tell driver we can't see the target
    }

    var drivetrain = subsystems.drivetrain;
    var aprilTag = subsystems.aprilTag.get();

    var targetID = AprilTagSubsystem.getAmpAprilTagID();
    var targetOptional = aprilTag.getTarget(targetID);
    if (targetOptional.isEmpty()) {
      return Commands.none(); // TODO blink LEDS red to tell driver we can't see the target
    }

    var target = targetOptional.get();
    var cameraToTarget = target.getBestCameraToTarget();
    Pose3d targetPose;

    if (USE_ESTIMATED_POSE.getValue()) {
      targetPose = aprilTag.getAprilTagPose(targetID);
    } else {
      // Transform the robot's pose to find the tag's pose
      var robotPose = drivetrain.getPosition3d();
      var cameraPose = robotPose.transformBy(RobotConstants.APRILTAG_CAMERA_TO_ROBOT);
      System.out.println("CAMERA POSE = " + cameraPose);
      targetPose = cameraPose.transformBy(cameraToTarget);
    }

    System.out.println("TARGET POSE = " + targetPose);

    // Find the scoring position pose.
    var tagToGoal =
        new Transform3d(
            new Translation3d(RobotConstants.SCORING_DISTANCE_FROM_AMP, 0, 0.0),
            new Rotation3d(0, 0, Math.PI));
    var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

    System.out.println("GOAL POSE = " + goalPose);

    return Commands.sequence(
        new DriveStraight(drivetrain, goalPose, SwerveSubsystem.getMaxSpeed() * 0.5));
  }

  /**
   * Returns a command that resets the drivetrain orientation for teleop driving based on the
   * current alliance.
   *
   * @param subsystems The subsystems container.
   * @return A command to reset the drivetrain orientation.
   */
  public static Command resetOrientation(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;

    return Commands.select(
        Map.of(
            Alliance.Blue,
            Commands.runOnce(() -> drivetrain.resetOrientation(new Rotation2d()), drivetrain),
            Alliance.Red,
            Commands.runOnce(
                () -> drivetrain.resetOrientation(Rotation2d.fromDegrees(180)), drivetrain)),
        () -> {
          Optional<Alliance> alliance = DriverStation.getAlliance();
          return alliance.orElse(Alliance.Blue);
        });
  }

  public static Command enablePoseEstimation(Subsystems subsystems, boolean enable) {
    Optional<AprilTagSubsystem> aprilTag = subsystems.aprilTag;
    return Commands.runOnce(
        () -> {
          if (aprilTag.isPresent()) {
            Subsystems.ENABLE_POSE_ESTIMATION.setValue(enable);
          }
        });
  }

  /**
   * Returns a command that enables auto orientation to the current alliance speaker.
   *
   * @param subsystems The subsystem container.
   * @return
   */
  public static Command autoOrientToSpeaker(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    Optional<AprilTagSubsystem> aprilTag = subsystems.aprilTag;
    return Commands.runOnce(
        () -> {
          if (aprilTag.isPresent()) {
            drivetrain.enableAutoOrientation(
                () -> {
                  Optional<PhotonTrackedTarget> target = aprilTag.get().getSpeakerCenterAprilTag();

                  if (target.isPresent()) {
                    // The speaker center AprilTag is visible, so use the relative angle reported by
                    // PhotonVision to determine the desired heading.
                    double angle = PhotonVisionSubsystemBase.calculateAngleToTarget(target.get());
                    double currentOrientation = drivetrain.getOrientation().getRadians();
                    double targetOrientation = MathUtil.angleModulus(currentOrientation + angle);

                    return Optional.of(new Rotation2d(targetOrientation));
                  } else {
                    // The speaker center AprilTag is not visible, so orient the robot to the
                    // absolute location relative to the current estimated robot pose.
                    Translation2d aprilTagLocation =
                        subsystems
                            .aprilTag
                            .get()
                            .getSpeakerCenterAprilTagPose()
                            .toPose2d()
                            .getTranslation();
                    return Optional.of(
                        aprilTagLocation
                            .minus(drivetrain.getPosition().getTranslation())
                            .getAngle()
                            .rotateBy(ROTATE_180_DEGREES));
                  }
                });
          }
        });
  }

  /**
   * Returns a command to enable auto orient to note mode.
   *
   * @param subsystems The subsystems container.
   * @return
   */
  public static Command autoOrientToNote(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    Optional<NoteVisionSubsystem> noteVision = subsystems.noteVision;
    return Commands.runOnce(
        () -> {
          if (noteVision.isPresent()) {
            drivetrain.enableAutoOrientation(
                () -> {
                  double currentOrientation = drivetrain.getOrientation().getRadians();
                  double angleToTarget = Math.toRadians(noteVision.get().getAngleToBestTarget());
                  double targetOrientation =
                      MathUtil.angleModulus(angleToTarget + currentOrientation);

                  return Optional.of(new Rotation2d(targetOrientation));
                });
          }
        });
  }

  /**
   * Returns a command that disables auto orientation.
   *
   * @param subsystems The subsystems container.
   * @return
   */
  public static Command disableAutoOrientation(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    return Commands.runOnce(() -> drivetrain.disableAutoOrientation());
  }
}
