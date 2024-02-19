/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Subsystems;

/** Add your docs here. */
public class ShooterCommands {
  /**
   * Returns a command to set the goal RPM of the shooter.
   *
   * @param subsystems The subsystems container.
   * @param rpm The desired RPM.
   * @return The command.
   */
  public static Command setRPM(Subsystems subsystems, double rpm) {
    ShooterSubsystem shooter = subsystems.shooter;
    return Commands.runOnce(() -> shooter.setGoalRPM(rpm), shooter);
  }

  /**
   * Returns command sequence to set and wait for shooter to reach the specified RPM.
   *
   * @param subsystems The subsystems container.
   * @param rpm The desired RPM.
   * @return The command sequence.
   */
  public static Command setAndWaitForRPM(Subsystems subsystems, double rpm) {
    ShooterSubsystem shooter = subsystems.shooter;
    return Commands.sequence( //
        setRPM(subsystems, rpm), //
        Commands.idle(shooter).until(shooter::atGoalRPM));
  }
}
