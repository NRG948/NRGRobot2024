/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.Subsystems;
import java.util.Optional;

/** Commands to wind and undwind the climber subsystem on the chain. */
public class ClimberCommands {

  /**
   * Returns a command to manually wind the climber.
   *
   * @param subsystems The subsystems container.
   * @return A command to manually wind the climber.
   */
  public static Command manualClimbChain(Subsystems subsystems) {
    Optional<ClimberSubsystem> climber = subsystems.climber;

    if (climber.isEmpty()) {
      return Commands.none();
    }

    return Commands.run(() -> climber.get().wind(), climber.get())
        .finallyDo(() -> climber.get().stopMotors());
  }

  /**
   * Returns a command to manually unwind the climber.
   *
   * @param subsystems The subsystems container.
   * @return A command to manually unwind the climber.
   */
  public static Command manualClimbDownChain(Subsystems subsystems) {
    Optional<ClimberSubsystem> climber = subsystems.climber;

    if (climber.isEmpty()) {
      return Commands.none();
    }

    return Commands.run(() -> climber.get().unwind(), climber.get())
        .finallyDo(() -> climber.get().stopMotors());
  }
}
