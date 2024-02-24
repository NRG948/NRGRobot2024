/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import com.nrg948.autonomous.AutonomousCommandGenerator;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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

    // Map<String, Command> map = getPathplannerEventMap(subsystems, name);
    // map.get(name);
    NamedCommands.registerCommands(getPathplannerEventMap(subsystems, name));
    // Command test = get

    return Commands.sequence(
        Commands.runOnce(() -> driveTrain.resetPosition(startPose), driveTrain),
        Commands.defer(() -> new PathPlannerAuto(name), Set.of(driveTrain)));
    // Commands.runOnce(() -> )
  }

  // public static void registerCommands(Map<String, Command> commands){
  // NamedCommands.re;
  // }

  private static Map<String, Command> getPathplannerEventMap(
      Subsystems subsystems, String pathGroupName) {

    Map<String, Command> eventMaps =
        new HashMap<String, Command>(); // TODO: Replace placeholder parameters
    eventMaps.put("SetShooterRPMSpike", ShooterCommands.setAndWaitForRPM(subsystems, 2000));
    eventMaps.put("SetShooterRPMAmpFarShot", ShooterCommands.setAndWaitForRPM(subsystems, 5000));
    eventMaps.put("SetShooterRPMSourceFarShot", ShooterCommands.setAndWaitForRPM(subsystems, 5000));
    eventMaps.put("SetArmAngleSpike", ArmCommands.seekToAngle(subsystems, Math.toRadians(15)));
    eventMaps.put("SetArmAngleAmpFarShot", ArmCommands.seekToAngle(subsystems, Math.toRadians(10)));
    eventMaps.put(
        "SetArmAngleSourceFarShot", ArmCommands.seekToAngle(subsystems, Math.toRadians(10)));
    eventMaps.put("FeedIndexerFullPower", Commands.runOnce(() -> subsystems.indexer.feed()));
    eventMaps.put("StowArm", ArmCommands.stow(subsystems));
    eventMaps.put("Intake", NoteCommands.intake(subsystems)); // will run our intake sequence

    return eventMaps;
  }

  public static void NamedCommands() {}

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
