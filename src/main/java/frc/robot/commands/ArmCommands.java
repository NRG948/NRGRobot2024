/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Subsystems;
import java.util.Set;

/** Add your docs here. */
public class ArmCommands {

  /**
   * Returns a command to move the arm to the stowed position.
   *
   * @param subsystems The subsystems container.
   * @return A command to move the arm to to stowed position.
   */
  public static Command stow(Subsystems subsystems) {
    return Commands.none();
    // return seekToAngle(subsystems, ArmSubsystem.STOWED_ANGLE);
  }

  /**
   * Returns a command to move the arm to the amp position.
   *
   * @param subsystems The subsystems container.
   * @return A command to move the arm to to amp position.
   */
  public static Command seekToAmp(Subsystems subsystems) {
    return Commands.defer(
        () -> seekToAngle(subsystems, Math.toRadians(ArmSubsystem.AMP_ANGLE.getValue())),
        Set.of(subsystems.arm));
  }

  /**
   * Returns a command to move the arm to the trap position.
   *
   * @param subsystems The subsystems container.
   * @return A command to move the arm to to trap position.
   */
  public static Command seekToTrap(Subsystems subsystems) {
    return seekToAngle(subsystems, Math.toRadians(ArmSubsystem.TRAP_ANGLE.getValue()));
  }

  /**
   * Returns a command to move the arm to the specified angle.
   *
   * @param subsystems The subsystems container.
   * @return A command to move the arm to to trap position.
   */
  public static Command seekToAngle(Subsystems subsystems, double angle) {
    ArmSubsystem arm = subsystems.arm;

    return Commands.none();
    // return Commands.sequence(
    //     Commands.runOnce(() -> arm.setGoalAngle(angle), arm), //
    //     Commands.idle(arm)); // Suppress default command
  }

  /**
   * Disables automatic seek to goal angle.
   *
   * @param subsystems The subsystems container.
   * @return A command to disable automatic seeking to goal angle.
   */
  public static Command disableSeek(Subsystems subsystems) {
    ArmSubsystem arm = subsystems.arm;

    return Commands.runOnce(() -> arm.disable(), arm);
  }
}
