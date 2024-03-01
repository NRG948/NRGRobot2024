/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class RainbowLEDCycle extends Command {
  public static int colorIndex = 0;
  private final LEDSubsystem led;

  /** Creates a new RainbowLEDCycle. */
  public RainbowLEDCycle(LEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.fillAndCommitColor(Constants.ColorConstants.COLORS[colorIndex]);
    if (colorIndex >= Constants.ColorConstants.COLORS.length - 1) {
      colorIndex = 0;
    } else {
      colorIndex += 1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
