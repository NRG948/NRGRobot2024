/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Subsystems;

public class NoteCommands {

  private static final double EXTRA_SHOT_DELAY = 0.2;

  /** The initial intake time when autoCenterNote is in sequence with intakeUntilNoteDetected. */
  public static double AUTO_CENTER_NOTE_CONTINUATION = 0;

  /** The inital intake time when autoCenterNote is used standalone. */
  public static double AUTO_CENTER_NOTE_STANDALONE = 0.1;

  /** The final delay for intaking the note during auto centering. */
  private static final double FINAL_INTAKE_SECONDS = 0.1;

  /**
   * Returns a sequence of commands to intake the note into the indexer until interrupted by another
   * command.
   *
   * <p>This command should be bound to a button using {@link Trigger#whileTrue}.
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
   * <p>This command should be bound to a button using {@link Trigger#whileTrue}.
   *
   * @param subsystems The subsystems container.
   * @return The command sequence.
   */
  public static Command intakeUntilNoteDetected(Subsystems subsystems) {
    return intakeUntilNoteDetected(subsystems, true);
  }

  /**
   * Returns a sequence of commands to intake the note into the indexer until beam break is
   * triggered.
   *
   * <p>This method is intended to be used to create a sequence with another command like {@link
   * NoteCommands#autoCenterNote(Subsystems)} that requires the indexer to be enabled.
   *
   * @param subsystems The subsystems container.
   * @param disableIndexer If true, disables the indexer at the end of the command.
   * @return The command sequence.
   */
  public static Command intakeUntilNoteDetected(Subsystems subsystems, boolean disableIndexer) {
    IntakeSubsystem intake = subsystems.intake;
    IndexerSubsystem indexer = subsystems.indexer;

    return intake(subsystems)
        .until(indexer::isNoteBreakingPracticeBeam) //
        .finallyDo(
            () -> {
              intake.disable();
              if (disableIndexer) {
                indexer.disable();
              }
            });
  }

  /**
   * Returns a sequence of commands to intake the note into the indexer until beam break is
   * triggered and then auto center the note.
   *
   * <p>This command should be bound to a button using {@link Trigger#onTrue} to prevent
   * interrupting the auto-centering sequence.
   *
   * @param subsystems The subsystems container.
   * @return The command sequence.
   */
  public static Command intakeAndAutoCenterNote(Subsystems subsystems) {
    return Commands.sequence(
        NoteCommands.intakeUntilNoteDetected(subsystems, false),
        NoteCommands.autoCenterNote(subsystems, NoteCommands.AUTO_CENTER_NOTE_CONTINUATION));
  }

  /**
   * Returns a sequence of commands to automatically center the note.
   *
   * <p>This command is intended to be used standalone (i.e. not in sequence with an intake
   * command).
   *
   * @param subsystems The subsystems container.
   * @return The command sequence.
   */
  public static Command autoCenterNote(Subsystems subsystems) {
    return autoCenterNote(subsystems, AUTO_CENTER_NOTE_STANDALONE);
  }

  /**
   * Returns a sequence of commands to automatically center the note.
   *
   * <p>This method is intended to be used to create a sequence with another command like {@link
   * NoteCommands#intakeUntilNoteDetected(Subsystems)}.
   *
   * @param subsystems The subsystems container.
   * @param initialIntakeSeconds The initial intake time in seconds.
   * @return The command sequence.
   */
  public static Command autoCenterNote(Subsystems subsystems, double initialIntakeSeconds) {
    IndexerSubsystem indexer = subsystems.indexer;
    return Commands.sequence(
            Commands.runOnce(
                () -> indexer.intake(IndexerSubsystem.AUTO_CENTER_VELOCITY.getValue()), indexer),
            Commands.idle(indexer).withTimeout(initialIntakeSeconds),
            Commands.run(
                    () -> indexer.outtake(IndexerSubsystem.AUTO_CENTER_VELOCITY.getValue()),
                    indexer)
                .until(() -> !indexer.isNoteBreakingPracticeBeam()),
            Commands.runOnce(
                () -> indexer.intake(IndexerSubsystem.AUTO_CENTER_VELOCITY.getValue()),
                indexer), //
            Commands.idle(indexer).withTimeout(FINAL_INTAKE_SECONDS)) //
        .finallyDo(
            () -> {
              indexer.disable();
            });
  }

  /***
   * Returns a sequence of commands to outtake a note to the Amp.
   *
   * This command should be bound to a button using {@link Trigger#whileTrue}.
   *
   * @param subsystems The subsystems container
   * @return The command sequence.
   */

  public static Command outakeToAmp(Subsystems subsystems) {
    IndexerSubsystem indexer = subsystems.indexer;
    return Commands.sequence(
            Commands.runOnce(indexer::outtakeToAmp, indexer), Commands.idle(indexer))
        .finallyDo(indexer::disable);
  }

  /**
   * Returns a sequence of commands to outtake the note.
   *
   * <p>Assumes this command will be bound to a button using {@link Trigger#whileTrue}.
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
                    .until(() -> !indexer.isNoteBreakingPracticeBeam()),
                Commands.idle(shooter, indexer) // Suppress default commands
                    .withTimeout(EXTRA_SHOT_DELAY))
            .finallyDo(
                () -> {
                  shooter.disable();
                  indexer.disable();
                }), //
        Commands.none(), //
        indexer::isNoteBreakingPracticeBeam);
  }

  /***
   * Returns a command sequence to shoot the note at the current goal RPM.
   * @param subsystems The subsystems container.
   * @return The command sequence.
   */
  public static Command shootAtCurrentRPM(Subsystems subsystems) {
    IndexerSubsystem indexer = subsystems.indexer;
    ShooterSubsystem shooter = subsystems.shooter;
    return Commands.either( //
        Commands.sequence( //
                Commands.runOnce(indexer::feed, indexer), //
                Commands.idle(indexer, shooter) // Suppress default commands
                    .until(() -> !indexer.isNoteBreakingPracticeBeam()),
                Commands.idle(indexer, shooter) // Suppress default commands
                    .withTimeout(EXTRA_SHOT_DELAY))
            .finallyDo(
                () -> {
                  indexer.disable();
                  shooter.disable();
                }), //
        Commands.none(), //
        indexer::isNoteBreakingPracticeBeam);
  }

  /**
   * Sets the arm angle and the shooter RPM in preparation for the shot.
   *
   * <p>This command should be bound to a button using {@link Trigger#whileTrue(Command)}.
   *
   * @param subsystems The subsystems container.
   * @param rpm The desired shooter RPM.
   * @param goalAngle The desired arm angle.
   * @return The command sequence.
   */
  public static Command prepareToShoot(Subsystems subsystems, double rpm, double goalAngle) {
    ShooterSubsystem shooter = subsystems.shooter;
    ArmSubsystem arm = subsystems.arm;

    return Commands.sequence(
        Commands.parallel(
            // ArmCommands.seekToAngle(subsystems, goalAngle).until(arm::atGoalAngle),
            ShooterCommands.setRPM(subsystems, rpm)),
        Commands.idle(arm, shooter) // Suppress default commands
        );
  }
}
