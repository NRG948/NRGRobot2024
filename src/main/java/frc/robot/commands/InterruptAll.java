/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystems;

public class InterruptAll extends Command {
  private Subsystems subsystems;

  /** Creates a new InterruptAll. */
  public InterruptAll(Subsystems subsystems) {
    this.subsystems = subsystems;
    addRequirements(subsystems.all);
    subsystems.drivetrain.disableAutoOrientation();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystems.intake.disable();
    subsystems.indexer.disable();
    subsystems.shooter.disable();
    subsystems.arm.disable();
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
    return true;
  }
}
