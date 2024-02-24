/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import com.nrg948.autonomous.AutonomousCommandGenerator;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue SOURCE_FARSHOT_RPM =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Source Far Shot RPM", 4500); // TODO: RPM needs to be tested.

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue AMP_FARSHOT_RPM =
      new RobotPreferences.DoubleValue("Autonomous", "Amp Far Shot RPM", 4500); //TODO: RPM needs to be tested

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue SPIKE_SHOT_RPM =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Spike Shot RPM", 2000); // TODO: RPM needs to be tested.

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue MID_SPIKE_SHOT_RPM =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Mid Spike Shot RPM", 2000); // TODO: RPM needs to be tested.

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue SOURCE_FARSHOT_ANGLE =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Source Far Shot Angle", 10); // YODO: Angle needs to be tested.

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue AMP_FARSHOT_ANGLE =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Amp Far Shot Angle", 22); // TODO: Angle needs to be tested.

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue SPIKE_SHOT_ANGLE =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Spike Shot Angle", 12); // TODO: Angle needs to be tested.

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue MID_SPIKE_SHOT_ANGLE =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Mid Spike Shot Angle", 10); // TODO: Angle needs to be tested.

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue SHOOT_SOURCE_FARSHOT =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Source Far Shot Angle", 10); // YODO: Angle needs to be tested.

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue SHOOT_AMP_FARSHOT =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Amp Far Shot Angle", 22); // TODO: Angle needs to be tested.

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue SHOOT_SPIKE_SHOT =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Spike Shot Angle", 12); // TODO: Angle needs to be tested.

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue SHOOT_MID_SPIKE_SHOT =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Mid Spike Shot Angle", 10); // TODO: Angle needs to be tested.

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

    NamedCommands.registerCommands(getPathplannerEventMap(subsystems, name));

    return Commands.sequence(
        Commands.runOnce(() -> driveTrain.resetPosition(startPose), driveTrain),
        Commands.defer(() -> new PathPlannerAuto(name), Set.of(driveTrain)));
  }

  private static Map<String, Command> getPathplannerEventMap(
      Subsystems subsystems, String pathGroupName) {

    Map<String, Command> eventMaps =
        new HashMap<String, Command>(); // TODO: Replace placeholder parameters
    eventMaps.put("SetShooterRPMAmpFarShot", setShooterRPM(subsystems, AMP_FARSHOT_RPM));
    eventMaps.put("SetShooterRPMSourceFarShot", setShooterRPM(subsystems, SOURCE_FARSHOT_RPM));
    eventMaps.put("SetShooterRPMSpikeShot", setShooterRPM(subsystems, SPIKE_SHOT_RPM));
    eventMaps.put("SetShooterRPMMidSpikeShot", setShooterRPM(subsystems, MID_SPIKE_SHOT_RPM));

    eventMaps.put("SetArmAngleSpikeShot", setArmAngle(subsystems, SPIKE_SHOT_ANGLE));
    eventMaps.put("SetArmAngleAmpFarShot", setArmAngle(subsystems, AMP_FARSHOT_ANGLE));
    eventMaps.put("SetArmAngleSourceFarShot", setArmAngle(subsystems, SOURCE_FARSHOT_ANGLE));
    eventMaps.put("SetArmAngleMidSpikeShot", setArmAngle(subsystems, MID_SPIKE_SHOT_ANGLE));

    eventMaps.put("ShootSourceFarShot", shoot(subsystems, SHOOT_SOURCE_FARSHOT));
    eventMaps.put("ShootAmpFarShot", shoot(subsystems, SHOOT_AMP_FARSHOT));
    eventMaps.put("ShootSpikeShot", shoot(subsystems, SHOOT_SPIKE_SHOT));
    eventMaps.put("ShootMidSPikeShot", shoot(subsystems, SHOOT_MID_SPIKE_SHOT));

    eventMaps.put("FeedIndexerFullPower", Commands.runOnce(() -> subsystems.indexer.feed()));
    eventMaps.put("StowArm", ArmCommands.stow(subsystems));
    eventMaps.put("Intake", NoteCommands.intake(subsystems));
    eventMaps.put("IntakeUntilNoteDetected", NoteCommands.intakeUntilNoteDetected(subsystems));

    return eventMaps;
  }

  private static Command setShooterRPM(Subsystems subsystems, RobotPreferences.DoubleValue rpm) {
    ShooterSubsystem shooter = subsystems.shooter;
    return Commands.defer(
        () -> ShooterCommands.setRPM(subsystems, rpm.getValue()), Set.of(shooter));
  }

  private static Command setArmAngle(Subsystems subsystems, RobotPreferences.DoubleValue angle) {
    ArmSubsystem arm = subsystems.arm;
    return Commands.defer(
        () -> ArmCommands.seekToAngle(subsystems, Math.toRadians(angle.getValue())), Set.of(arm));
  }

  public static Command shoot(Subsystems subsystems, RobotPreferences.DoubleValue rpm) {
    ShooterSubsystem shooter = subsystems.shooter;
    IndexerSubsystem indexer = subsystems.indexer;
    return Commands.defer(
        () -> NoteCommands.shoot(subsystems, rpm.getValue()), Set.of(shooter, indexer));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
