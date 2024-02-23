/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import com.nrg948.autonomous.AutonomousCommandGenerator;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.javatuples.LabelValue;

public final class Autos {
  /**
   * Returns the collection of PathPlanner auto commands.
   *
   * @param subsystems The subsystem container.
   * @return A collection of PathPlanner auto commands.
   */
  @AutonomousCommandGenerator
  public static Collection<LabelValue<String, Command>> generatePathPlannerAutos(
      Subsystems subsystems) {
    File autosDir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
    return Arrays.stream(autosDir.listFiles((file, name) -> name.endsWith(".auto")))
        .map((file) -> file.getName().split("\\.")[0])
        .sorted()
        .map(name -> new LabelValue<>(name, getPathPlannerAuto(subsystems, name)))
        .toList();
  }

  /**
   * Returns the PathPlanner auto command.
   *
   * @param subsystems Subsystems container.
   * @param name Name of the PathPlanner auto.
   * @return The PathPlanner auto command.
   */
  public static Command getPathPlannerAuto(Subsystems subsystems, String name) {
    SwerveSubsystem driveTrain = subsystems.drivetrain;
    Pose2d startPose = PathPlannerAuto.getStaringPoseFromAutoFile(name);

    return Commands.sequence(
        Commands.runOnce(() -> driveTrain.resetPosition(startPose), driveTrain),
        Commands.defer(() -> new PathPlannerAuto(name), Set.of(driveTrain)));
  }

  private static Map<String, Command> getPathplannerEventMap(
      Subsystems subsystems, String pathGroupName, List<PathPlannerTrajectory> pathGroup) {
    Map<String, Command> eventMaps = new HashMap<String, Command>();

    eventMaps
        .put("SetShooterRPM", ShooterCommands.setAndWaitForRPM(subsystems, 0.0))
        .withTimeout(1);
    eventMaps.put("SetArmAngle", ArmCommands.seekToAngle(subsystems, 15)).withTimeout(1);
    eventMaps.put("", null);
    return eventMaps;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
