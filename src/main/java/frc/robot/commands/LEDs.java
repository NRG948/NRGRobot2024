// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AddressableLEDSubsystem;

/** Add your docs here. */
public class LEDs {
    public static Command fillColor(AddressableLEDSubsystem ledSubsystem, Color8Bit color) {
        return Commands.runOnce(() -> ledSubsystem.fillAndCommitColor(color), ledSubsystem);
    }
}
