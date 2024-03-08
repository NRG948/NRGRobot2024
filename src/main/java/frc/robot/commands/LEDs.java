/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ColorConstants;
import frc.robot.subsystems.LEDSubsystem;

/** Add your docs here. */
public class LEDs {
  public static Command fillColor(LEDSubsystem ledSubsystem, Color8Bit color) {
    return Commands.runOnce(() -> ledSubsystem.fillAndCommitColor(color), ledSubsystem);
  }

  public static Command flashColor(LEDSubsystem ledSubsystem, Color8Bit color) {
    return Commands.repeatingSequence(
            Commands.runOnce(() -> ledSubsystem.fillAndCommitColor(color)),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> ledSubsystem.fillAndCommitColor(ColorConstants.BLACK)),
            Commands.waitSeconds(0.1))
        .withTimeout(1);
  }
}
