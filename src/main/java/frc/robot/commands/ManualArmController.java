/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Subsystems;

public class ManualArmController extends Command {
  private final double ARM_POWER_LIMIT = 0.3;
  private final double DEADBAND = 0.05;

  private final CommandXboxController controller;
  private final ArmSubsystem arm;

  /** Creates a new ManualArmController. */
  public ManualArmController(Subsystems subsystems, CommandXboxController controller) {
    this.controller = controller;
    this.arm = subsystems.arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Start manual arm control");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -controller.getHID().getLeftY();
    speed = MathUtil.applyDeadband(speed, DEADBAND);
    arm.setMotorPowers(speed * ARM_POWER_LIMIT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("End manual arm control");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
