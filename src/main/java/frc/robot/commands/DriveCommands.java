// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import java.util.Optional;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotConstants;
import frc.robot.drive.SwerveDrive;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

public final class DriveCommands {
  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue USE_ESTIMATED_POSE = new RobotPreferences.BooleanValue("AprilTag", "Use Estimated Pose", false);

  /** Returns a Command that drives to amp delivery position. */
  public static Command driveToAmp(Subsystems subsystems) {
     
    var drivetrain = subsystems.drivetrain;
    var aprilTag = subsystems.aprilTag;

    var targetID = AprilTagSubsystem.getAmpAprilTagID();
    var targetOptional = aprilTag.getTarget(targetID);
    if(targetOptional.isEmpty()){
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
    var tagToGoal = new Transform3d(
        new Translation3d(RobotConstants.SCORING_DISTANCE_FROM_AMP, 0, 0.0),
        new Rotation3d(0, 0, Math.PI));
    var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

    System.out.println("GOAL POSE = " + goalPose);

    return Commands.sequence(
        new DriveStraight(drivetrain, goalPose, drivetrain.getMaxSpeed() * 0.5));
  }

  /**
   * Returns a command that resets the drivetrain orientation for teleop driving based on the current alliance.
   * 
   * @param subsystems The subsystems container.
   * @return A command to reset the drivetrain orientation.
   */
  public static Command resetOrientation(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;

    return Commands.select(
      Map.of(
        Alliance.Blue, Commands.runOnce(() -> drivetrain.resetOrientation(new Rotation2d()), drivetrain), 
        Alliance.Red, Commands.runOnce(() -> drivetrain.resetOrientation(Rotation2d.fromDegrees(180)), drivetrain)
      ), 
      () -> {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.orElse(Alliance.Blue);
      });

  }
}
