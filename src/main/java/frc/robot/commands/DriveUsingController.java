// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveUsingController extends Command {
  private final double DEADBAND = 0.05;

  private final SwerveSubsystem m_driveTrain;
  private final CommandXboxController m_xboxController;
  /** Creates a new DriveUsingController. */
  public DriveUsingController(SwerveSubsystem driveTrain, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_xboxController = xboxController;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -m_xboxController.getLeftY();
    double ySpeed = -m_xboxController.getLeftX();
    double rSpeed = -m_xboxController.getRightX();
    double inputScalar = Math.max(1.0-m_xboxController.getRightTriggerAxis(), 0.15);

    // Applies deadbands to x, y, and rotation joystick values and multiples all
    // values with inputSalar whihc allows finer driving control.
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * inputScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * inputScalar;
    rSpeed = MathUtil.applyDeadband(rSpeed, DEADBAND) * inputScalar;

    m_driveTrain.drive(
      xSpeed,
      ySpeed,
      rSpeed,
      true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
