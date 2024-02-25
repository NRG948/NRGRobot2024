/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Subsystems;

public class NoteCommands {
  /**
   * Returns a sequence of commands to intake the note into the indexer until interrupted by another
   * command.
   *
   * <p>This command should be bound to a button using whileTrue.
   *
   * @param subsystems The subsystems container.
   * @return The command sequence.
   */
  public static Command intake(Subsystems subsystems) {
    IntakeSubsystem intake = subsystems.intake;
    IndexerSubsystem indexer = subsystems.indexer;

    return Commands.sequence(
        Commands.parallel(
            Commands.runOnce(intake::in, intake), //
            Commands.runOnce(indexer::intake, indexer)), //
        Commands.idle(intake, indexer) // Supress default commands.
        );
  }

  /**
   * Returns a sequence of commands to intake the note into the indexer until beam break is
   * triggered.
   *
   * <p>This command should be bound to a button using whileTrue.
   *
   * @param subsystems The subsystems container.
   * @return The command sequence.
   */
  public static Command intakeUntilNoteDetected(Subsystems subsystems) {
    IntakeSubsystem intake = subsystems.intake;
    IndexerSubsystem indexer = subsystems.indexer;

    return Commands.sequence(
            intake(subsystems).until(indexer::isNoteDetected), //
            Commands.waitSeconds(0.2), //
            outtake(subsystems).until(() -> !indexer.isNoteDetected()), //
            intake(subsystems).until(indexer::isNoteDetected)) //
        .finallyDo(
            () -> {
              intake.disable();
              indexer.disable();
            });
  }

  /**
   * Returns a sequence of commands to outtake the note.
   *
   * <p>Assumes this command will be bound to a button using whileTrue.
   *
   * @param subsystems The subsystems container.
   * @return The command sequence.
   */
  public static Command outtake(Subsystems subsystems) {
    IntakeSubsystem intake = subsystems.intake;
    IndexerSubsystem indexer = subsystems.indexer;

    return Commands.sequence(
            Commands.parallel( //
                Commands.runOnce(intake::out, intake), //
                Commands.runOnce(indexer::outtake, indexer)), //
            Commands.idle(intake, indexer)) //
        .finallyDo(
            () -> {
              intake.disable();
              indexer.disable();
            });
  }

  /**
   * Returns a command sequence to shoot a note if one is in the indexer otherwise it will do
   * nothing.
   *
   * @param subsystems The subsystems container.
   * @param rpm The RPM to shoot the note.
   * @return The command sequence.
   */
  public static Command shoot(Subsystems subsystems, double rpm) {
    IndexerSubsystem indexer = subsystems.indexer;
    ShooterSubsystem shooter = subsystems.shooter;
    return Commands.either( //
        Commands.sequence( //
                ShooterCommands.setAndWaitForRPM(subsystems, rpm), //
                Commands.runOnce(indexer::feed, indexer), //
                Commands.idle(shooter, indexer) // Suppress default commands
                    .until(() -> !indexer.isNoteDetected()),
                Commands.idle(shooter, indexer) // Suppress default commands
                    .withTimeout(0.5))
            .finallyDo(
                () -> {
                  shooter.disable();
                  indexer.disable();
                }), //
        Commands.none(), //
        indexer::isNoteDetected);
  }
}
