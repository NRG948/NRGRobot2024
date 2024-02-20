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
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Subsystems;

public class IntakeUsingController extends Command {

  private final IntakeSubsystem intake;
  private final IndexerSubsystem indexer;
  private final CommandXboxController controller;

  private static final double DEADBAND = 0.1;

  /** Creates a new IntakeUsingController. */
  public IntakeUsingController(Subsystems subsystems, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = subsystems.intake; // need to uncomment in Subsystems.java
    this.indexer = subsystems.indexer;
    this.controller = controller;

    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -controller.getHID().getRightY();
    speed = MathUtil.applyDeadband(speed, DEADBAND);
    double voltage = speed * RobotConstants.MAX_BATTERY_VOLTAGE;
    intake.setMotorVoltage(voltage);
    indexer.setMotorVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
