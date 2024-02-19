/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;

/** Add your docs here. */
public class Pathfinding {
  private static final double PATH_SPEED_PERCENT = 0.2;
  private static final Pose2d BLUE_SPEAKER_FRONT =
      new Pose2d(new Translation2d(1.55, 5.52), new Rotation2d(0));
  private static final Pose2d RED_SPEAKER_FRONT =
      new Pose2d(new Translation2d(15.3, 5.52), Rotation2d.fromDegrees(180));
  private static final Pose2d BLUE_AMP_FRONT =
      new Pose2d(new Translation2d(1.90, 7.37), Rotation2d.fromDegrees(-90));
  private static final Pose2d RED_AMP_FRONT =
      new Pose2d(new Translation2d(14.66, 7.37), Rotation2d.fromDegrees(-90));

  /**
   * Returns a command in which the robot will pathfind to the front of the blue speaker.
   *
   * @return A command in which the robot will pathfind to the front of the blue speaker.
   */
  public static Command pathFindToSpeakerFront() {
    PathConstraints constraints =
        new PathConstraints(
            PATH_SPEED_PERCENT * SwerveSubsystem.getMaxSpeed(),
            PATH_SPEED_PERCENT * SwerveSubsystem.getMaxAcceleration(),
            PATH_SPEED_PERCENT * SwerveSubsystem.getRotationalConstraints().maxVelocity,
            PATH_SPEED_PERCENT * SwerveSubsystem.getRotationalConstraints().maxAcceleration);

    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      return Commands.none();
    }

    Pose2d robotPose = alliance.get() == Alliance.Blue ? BLUE_SPEAKER_FRONT : RED_SPEAKER_FRONT;

    return AutoBuilder.pathfindToPose(robotPose, constraints);
  }

  /**
   * Returns a command in which the robot will pathfind to the front of the blue amp.
   *
   * @return A command in which the robot will pathfind to the front of the blue amp.
   */
  public static Command pathFindToAmp() {
    PathConstraints constraints =
        new PathConstraints(
            PATH_SPEED_PERCENT * SwerveSubsystem.getMaxSpeed(),
            PATH_SPEED_PERCENT * SwerveSubsystem.getMaxAcceleration(),
            PATH_SPEED_PERCENT * SwerveSubsystem.getRotationalConstraints().maxVelocity,
            PATH_SPEED_PERCENT * SwerveSubsystem.getRotationalConstraints().maxAcceleration);

    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      return Commands.none();
    }

    Pose2d robotPose = alliance.get() == Alliance.Blue ? BLUE_AMP_FRONT : RED_AMP_FRONT;

    System.out.println("AMP Pose " + robotPose);

    return AutoBuilder.pathfindToPose(robotPose, constraints);
  }

  /**
   * Returns a command in which the robot will pathfind to the front of the blue amp.
   *
   * @return A command in which the robot will pathfind to the front of the blue amp.
   */
  public static Command pathFindToAmp2() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Amp Test Path");
    return AutoBuilder.followPath(path);
  }

  private Pathfinding() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
