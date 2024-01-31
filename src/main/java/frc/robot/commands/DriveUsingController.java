// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveUsingController extends Command {
  private static final double DEADBAND = 0.05;
  private static final double KP = 1.0;

  private final SwerveSubsystem m_drivetrain;
  private final AprilTagSubsystem m_aprilTag;
  private final CommandXboxController m_xboxController;
  private ProfiledPIDController m_profiledPIDController;

 

  /** Creates a new DriveUsingController. */
  public DriveUsingController(Subsystems subsystems, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = subsystems.drivetrain;
    m_aprilTag = subsystems.aprilTag;
    m_xboxController = xboxController;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_profiledPIDController = new ProfiledPIDController(KP, 0, 0, m_drivetrain.getRotationalConstraints());
    m_profiledPIDController.reset(m_drivetrain.getOrientation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rSpeed;
    double xSpeed = -m_xboxController.getLeftY();
    double ySpeed = -m_xboxController.getLeftX();
    double inputScalar = Math.max(1.0-m_xboxController.getRightTriggerAxis(), 0.15);

    Rotation2d currentOrientation = m_drivetrain.getOrientation();
    Rotation2d targetOrientation = currentOrientation;

    // Applies deadbands to x and y joystick values and multiples all
    // values with inputScalar whihc allows finer driving control.
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * inputScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * inputScalar;

    Optional<PhotonTrackedTarget> optionalTarget = Optional.empty();
    if (m_xboxController.rightBumper().getAsBoolean()) {
      optionalTarget = m_aprilTag.getTarget(AprilTagSubsystem.getSpeakerCenterAprilTagID()); 
    }

    if (optionalTarget.isPresent()) {
      Rotation2d angleToTarget = Rotation2d.fromDegrees(-m_aprilTag.getAngleToBestTarget());
      targetOrientation = targetOrientation.plus(angleToTarget);
      rSpeed = m_profiledPIDController.calculate(currentOrientation.getRadians(), targetOrientation.getRadians());
    } else {
      rSpeed = -m_xboxController.getRightX();
      rSpeed = MathUtil.applyDeadband(rSpeed, DEADBAND) * inputScalar;
    } 

    m_drivetrain.drive(
      xSpeed,
      ySpeed,
      rSpeed,
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
