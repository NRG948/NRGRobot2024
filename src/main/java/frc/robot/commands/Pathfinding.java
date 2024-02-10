// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class Pathfinding {
	private static final double PATH_SPEED_PERCENT = 0.5;
	private static final Pose2d BLUE_SPEAKER_FRONT = new Pose2d(
			new Translation2d(1.55, 5.52),
			new Rotation2d(0));

	/**
	 * Returns a command in which the robot will pathfind to the front of the blue speaker.
	 * 
	 * @return A command in which the robot will pathfind to the front of the blue speaker.
	 */
	public static Command pathFindToSpeakerFront() {
		PathConstraints constraints = new PathConstraints( 
			PATH_SPEED_PERCENT * SwerveSubsystem.getMaxSpeed(),
			PATH_SPEED_PERCENT * SwerveSubsystem.getMaxAcceleration(),
			PATH_SPEED_PERCENT * SwerveSubsystem.getRotationalConstraints().maxVelocity,
			PATH_SPEED_PERCENT * SwerveSubsystem.getRotationalConstraints().maxAcceleration);
		return AutoBuilder.pathfindToPose(BLUE_SPEAKER_FRONT, constraints);
	}

	private Pathfinding() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
