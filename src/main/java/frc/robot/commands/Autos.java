/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import com.nrg948.autonomous.AutonomousCommandGenerator;
import com.nrg948.autonomous.AutonomousCommandMethod;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

@RobotPreferencesLayout(
    groupName = "Autonomous",
    column = 7,
    row = 0,
    width = 2,
    height = 4,
    type = "Grid Layout",
    gridColumns = 2,
    gridRows = 6)
public final class Autos {
  @RobotPreferencesValue(column = 0, row = 0)
  public static RobotPreferences.DoubleValue SOURCE_FARSHOT_RPM =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Source Far Shot RPM", 3575); // TODO: RPM needs to be tested.

  @RobotPreferencesValue(column = 1, row = 0)
  public static RobotPreferences.DoubleValue SOURCE_FARSHOT_ANGLE =
      new RobotPreferences.DoubleValue("Autonomous", "Source Far Shot Angle", 22.5);

  @RobotPreferencesValue(column = 0, row = 1)
  public static RobotPreferences.DoubleValue AMP_FARSHOT_RPM =
      new RobotPreferences.DoubleValue("Autonomous", "Amp Far Shot RPM", 3425);

  @RobotPreferencesValue(column = 1, row = 1)
  public static RobotPreferences.DoubleValue AMP_FARSHOT_ANGLE =
      new RobotPreferences.DoubleValue("Autonomous", "Amp Far Shot Angle", 20);

  @RobotPreferencesValue(column = 0, row = 2)
  public static RobotPreferences.DoubleValue SPIKE_SHOT_RPM =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Spike Shot RPM", 3000); // TODO: RPM needs to be tested.

  @RobotPreferencesValue(column = 1, row = 2)
  public static RobotPreferences.DoubleValue SPIKE_SHOT_ANGLE =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Spike Shot Angle", 14.5); // TODO: Angle needs to be tested.

  @RobotPreferencesValue(column = 0, row = 3)
  public static RobotPreferences.DoubleValue MID_SPIKE_SHOT_RPM =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Mid Spike Shot RPM", 3000); // TODO: RPM needs to be tested.

  @RobotPreferencesValue(column = 1, row = 3)
  public static RobotPreferences.DoubleValue MID_SPIKE_SHOT_ANGLE =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Mid Spike Shot Angle", 10.5); // TODO: Angle needs to be tested.

  @RobotPreferencesValue(column = 0, row = 4)
  public static RobotPreferences.DoubleValue SUBWOOFER_SIDE_SHOT_RPM =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Subwoofer Side Shot RPM", 2200); // TODO: RPM needs to be tested.

  @RobotPreferencesValue(column = 1, row = 4)
  public static RobotPreferences.DoubleValue SUBWOOFER_SIDE_SHOT_ANGLE =
      new RobotPreferences.DoubleValue("Autonomous", "Subwoofer Side Shot Angle", -11);

  @RobotPreferencesValue(column = 0, row = 5)
  public static RobotPreferences.DoubleValue SUBWOOFER_SHOT_RPM =
      new RobotPreferences.DoubleValue(
          "Autonomous", "Subwoofer Shot RPM", 2200); // TODO: RPM needs to be tested.

  @RobotPreferencesValue(column = 1, row = 5)
  public static RobotPreferences.DoubleValue SUBWOOFER_SHOT_ANGLE =
      new RobotPreferences.DoubleValue("Autonomous", "Subwoofer Shot Angle", -11);

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
    try {
      SwerveSubsystem driveTrain = subsystems.drivetrain;
      Pose2d startPose = PathPlannerAuto.getStaringPoseFromAutoFile(name);

      NamedCommands.registerCommands(getPathplannerEventMap(subsystems, name));

      return Commands.sequence(
          Commands.runOnce(() -> driveTrain.resetPosition(startPose), driveTrain),
          Commands.defer(() -> new PathPlannerAuto(name), Set.of(driveTrain)));
    } catch (Exception e) {
      System.out.println("ERROR: Failed to load PathPlanner file " + name + ": " + e.getMessage());
      return Commands.none();
    }
  }

  private static Map<String, Command> getPathplannerEventMap(
      Subsystems subsystems, String pathGroupName) {

    Map<String, Command> eventMaps =
        new HashMap<String, Command>(); // TODO: Replace placeholder parameters
    eventMaps.put("SetShooterRPMAmpFarShot", setShooterRPM(subsystems, AMP_FARSHOT_RPM));
    eventMaps.put("SetShooterRPMSourceFarShot", setShooterRPM(subsystems, SOURCE_FARSHOT_RPM));
    eventMaps.put("SetShooterRPMSpikeShot", setShooterRPM(subsystems, SPIKE_SHOT_RPM));
    eventMaps.put("SetShooterRPMMidSpikeShot", setShooterRPM(subsystems, MID_SPIKE_SHOT_RPM));
    eventMaps.put(
        "SetShooterRPMSubwooferSideShot", setShooterRPM(subsystems, SUBWOOFER_SIDE_SHOT_RPM));
    eventMaps.put("SetShooterRPMSubwooferShot", setShooterRPM(subsystems, SUBWOOFER_SHOT_RPM));
    eventMaps.put("SetSubwooferRPMAndWait", setShooterRPMAndWait(subsystems, SUBWOOFER_SHOT_RPM));

    eventMaps.put("SetArmAngleSpikeShot", setArmAngle(subsystems, SPIKE_SHOT_ANGLE));
    eventMaps.put("SetArmAngleAmpFarShot", setArmAngle(subsystems, AMP_FARSHOT_ANGLE));
    eventMaps.put("SetArmAngleSourceFarShot", setArmAngle(subsystems, SOURCE_FARSHOT_ANGLE));
    eventMaps.put("SetArmAngleMidSpikeShot", setArmAngle(subsystems, MID_SPIKE_SHOT_ANGLE));
    eventMaps.put(
        "SetArmAngleSubwooferSideShot", setArmAngle(subsystems, SUBWOOFER_SIDE_SHOT_ANGLE));
    eventMaps.put("SetArmAngleSubwooferShot", setArmAngle(subsystems, SUBWOOFER_SHOT_ANGLE));
    eventMaps.put("SetShooterContinuous", new SetShooterContinous(subsystems));

    eventMaps.put("ShootSourceFarShot", shoot(subsystems, SOURCE_FARSHOT_RPM));
    eventMaps.put("ShootAmpFarShot", shoot(subsystems, AMP_FARSHOT_RPM));
    eventMaps.put("ShootSpikeShot", shoot(subsystems, SPIKE_SHOT_RPM));
    eventMaps.put("ShootMidSPikeShot", shoot(subsystems, MID_SPIKE_SHOT_RPM));
    eventMaps.put("ShootSubwooferSideShot", shoot(subsystems, SUBWOOFER_SIDE_SHOT_RPM));
    eventMaps.put("ShootSubwooferShot", shoot(subsystems, SUBWOOFER_SHOT_RPM));
    eventMaps.put("ShootAtCurrentRPM", NoteCommands.shootAtCurrentRPM(subsystems));

    eventMaps.put("SetShooterContinous", setShooterContinous(subsystems));
    eventMaps.put("FeedIndexerFullPower", Commands.runOnce(() -> subsystems.indexer.feed()));
    eventMaps.put("StowArm", ArmCommands.stow(subsystems));
    eventMaps.put("Intake", NoteCommands.intake(subsystems));
    eventMaps.put("IntakeUntilNoteDetected", autoIntakeNote(subsystems));
    eventMaps.put(
        "AutoCenterNote",
        NoteCommands.autoCenterNote(subsystems, NoteCommands.AUTO_CENTER_NOTE_CONTINUATION));
    eventMaps.put(
        "IntakeUntilNoteDetectedNoAutoCentering", NoteCommands.intakeUntilNoteDetected(subsystems));

    eventMaps.put("AutoOrientToSpeaker", DriveCommands.autoOrientToSpeaker(subsystems));
    eventMaps.put("DisableAutoOrientation", DriveCommands.disableAutoOrientation(subsystems));
    eventMaps.put("EnablePoseEstimation", DriveCommands.enablePoseEstimation(subsystems, true));
    eventMaps.put("DisablePoseEstimation", DriveCommands.enablePoseEstimation(subsystems, false));

    eventMaps.put("AutoSetShooter", autoSetShooter(subsystems));
    return eventMaps;
  }

  /**
   * Returns a command sequence to set the shooter RPM using the specified preferences value.
   *
   * <p>This command is intended to be used in a PathPlanner auto.
   *
   * @param subsystems The subsystems container.
   * @param rpm The {@link RobotPreferences.DoubleValue} containing the RPM value.
   * @return A command sequence to set the shooter RPM.
   */
  private static Command setShooterRPM(Subsystems subsystems, RobotPreferences.DoubleValue rpm) {
    ShooterSubsystem shooter = subsystems.shooter;
    return Commands.defer(
        () -> ShooterCommands.setRPM(subsystems, rpm.getValue()), Set.of(shooter));
  }

  public static Command setShooterRPMAndWait(
      Subsystems subsystems, RobotPreferences.DoubleValue rpm) {
    ShooterSubsystem shooter = subsystems.shooter;
    return Commands.defer(
        () -> ShooterCommands.setAndWaitForRPM(subsystems, rpm.getValue()), Set.of(shooter));
  }

  /**
   * Returns a command sequence to set the arm angle using the specified preferences value.
   *
   * <p>This command is intended to be used in a PathPlanner auto.
   *
   * @param subsystems The subsystems container.
   * @param angle The {@link RobotPreferences.DoubleValue} containing the angle value.
   * @return A command sequence to set the arm angle.
   */
  private static Command setArmAngle(Subsystems subsystems, RobotPreferences.DoubleValue angle) {
    ArmSubsystem arm = subsystems.arm;
    return Commands.defer(
        () ->
            ArmCommands.seekToAngle(subsystems, Math.toRadians(angle.getValue()))
                .until(arm::atGoalAngle),
        Set.of(arm));
  }

  /**
   * Returns a command sequence to shoot a note if one is in the indexer otherwise it will do
   * nothing.
   *
   * <p>This command is intended to be used in a PathPlanner auto.
   *
   * @param subsystems The subsystems container.
   * @param rpm The RPM to shoot the note.
   * @return The command sequence.
   */
  public static Command shoot(Subsystems subsystems, RobotPreferences.DoubleValue rpm) {
    ShooterSubsystem shooter = subsystems.shooter;
    IndexerSubsystem indexer = subsystems.indexer;
    return Commands.defer(
        () -> NoteCommands.shoot(subsystems, rpm.getValue()), Set.of(shooter, indexer));
  }

  /**
   * Returns a command sequence to intake a note and then center it.
   *
   * <p>This command is intended to be used in a PathPlanner auto.
   *
   * @param subsystems The subsystems container.
   * @return The command sequence.
   */
  public static Command autoIntakeNote(Subsystems subsystems) {
    return Commands.sequence(
        NoteCommands.intakeUntilNoteDetected(subsystems, false),
        NoteCommands.autoCenterNote(subsystems, NoteCommands.AUTO_CENTER_NOTE_CONTINUATION));
  }

  public static Command alignWheels(Subsystems subsystems) {
    SwerveSubsystem swerveDrive = subsystems.drivetrain;
    SwerveModuleState moduleState = new SwerveModuleState();

    SwerveModuleState[] moduleStates =
        new SwerveModuleState[] {moduleState, moduleState, moduleState, moduleState};
    return Commands.runOnce(() -> swerveDrive.setModuleStates(moduleStates), swerveDrive);
  }

  public static Command setShooterContinous(Subsystems subsystems) {
    return new SetShooterContinous(subsystems);
  }

  public static Command autoSetShooter(Subsystems subsystems) {
    return setShooterContinous(subsystems)
        .until(() -> subsystems.arm.atGoalAngle() && subsystems.shooter.atGoalRPM());
  }

  @AutonomousCommandMethod(name = "No Auto")
  public static Command noAuto(Subsystems subsystems) {
    return Commands.none();
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
