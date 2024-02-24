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
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootUsingController extends Command {
  /** Creates a new ShootByController. */
  private ShooterSubsystem shooterSubsystem;

  private CommandXboxController controller;

  private static final double DEADBAND = 0.1;
  private static final double SHOOTER_SPEED = 1.0;

  public ShootUsingController(ShooterSubsystem shooterSubsystem, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shooterSubsystem.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = -controller.getHID().getLeftY();
    speed = MathUtil.applyDeadband(speed * SHOOTER_SPEED, DEADBAND);
    double voltage = speed * RobotConstants.MAX_BATTERY_VOLTAGE;
    shooterSubsystem.setMotorVoltages(voltage, voltage * ShooterSubsystem.SPIN_FACTOR.getValue());
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
