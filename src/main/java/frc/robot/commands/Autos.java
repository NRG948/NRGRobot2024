// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Subsystems;

import com.nrg948.autonomous.AutonomousCommandMethod;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;

public final class Autos {
  /** Example static factory for an autonomous command. */
  @AutonomousCommandMethod(name = "Award Winning Auto", isDefault = true)
  public static Command exampleAuto(Subsystems subsystem) {
    return new PathPlannerAuto("Award winning auto");
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
