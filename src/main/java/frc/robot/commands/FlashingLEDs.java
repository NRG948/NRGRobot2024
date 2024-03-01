/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class FlashingLEDs extends Command {
  private final Color8Bit color1;
  private final Color8Bit color2;
  private final LEDSubsystem led;
  private final Timer timer = new Timer();
  private final double blinkingSpeed;

  private Color8Bit color;

  /** Creates a new FlashingLEDs. */
  public FlashingLEDs(Color8Bit color1, Color8Bit color2, LEDSubsystem led, double blinkingSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.color1 = color1;
    this.color2 = color2;
    this.led = led;
    this.blinkingSpeed = blinkingSpeed;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.fillAndCommitColor(color1);
    timer.start();
    color = color1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.advanceIfElapsed(blinkingSpeed)) {
      if (color.equals(color1)) {
        color = color2;
      } else {
        color = color1;
      }
      led.fillAndCommitColor(color);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
