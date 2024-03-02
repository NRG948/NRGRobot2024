/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.NoteVisionSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DriveUsingController extends Command {
  private static final double DEADBAND = 0.08;

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue KP_APRIL_TAG =
      new RobotPreferences.DoubleValue("AprilTag", "kP", 1.0);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue KI_APRIL_TAG =
      new RobotPreferences.DoubleValue("AprilTag", "kI", 0);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue KD_APRIL_TAG =
      new RobotPreferences.DoubleValue("AprilTag", "kD", 0);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue KP_NOTE =
      new RobotPreferences.DoubleValue("NoteVision", "kP", 0.7);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue KI_NOTE =
      new RobotPreferences.DoubleValue("NoteVision", "kI", 0.0);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue KD_NOTE =
      new RobotPreferences.DoubleValue("NoteVision", "kD", 0.0);

  private final SwerveSubsystem drivetrain;
  private final Optional<AprilTagSubsystem> aprilTag;
  private final Optional<NoteVisionSubsystem> noteVision;
  private final CommandXboxController xboxController;
  private ProfiledPIDController profiledPIDController;

  /** Creates a new DriveUsingController. */
  public DriveUsingController(Subsystems subsystems, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = subsystems.drivetrain;
    aprilTag = subsystems.aprilTag;
    noteVision = subsystems.noteVision;
    this.xboxController = xboxController;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    profiledPIDController =
        new ProfiledPIDController(
            KP_APRIL_TAG.getValue(),
            KI_APRIL_TAG.getValue(),
            KD_APRIL_TAG.getValue(),
            SwerveSubsystem.getRotationalConstraints());
    profiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
    profiledPIDController.reset(drivetrain.getOrientation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rSpeed;
    double xSpeed = -xboxController.getLeftY();
    double ySpeed = -xboxController.getLeftX();
    double inputScalar = Math.max(1.0 - xboxController.getRightTriggerAxis(), 0.15);

    double currentOrientation = drivetrain.getOrientation().getRadians();
    double targetOrientation = currentOrientation;

    // Applies deadbands to x and y joystick values and multiples all
    // values with inputScalar whihc allows finer driving control.
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * inputScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * inputScalar;

    Optional<PhotonTrackedTarget> optionalTagTarget = Optional.empty();
    Optional<PhotonTrackedTarget> optionalNoteTarget = Optional.empty();

    if (xboxController.getHID().getRightStickButton() && aprilTag.isPresent()) {
      optionalTagTarget = aprilTag.get().getTarget(AprilTagSubsystem.getSpeakerCenterAprilTagID());
    } else if (xboxController.getHID().getXButton()
        && noteVision.isPresent()
        && noteVision.get().hasTargets()) { // Nonpermanent X binding
      optionalNoteTarget = Optional.of(noteVision.get().getBestTarget());
    }

    // Don't want to do both tag and note alignment so to choose one, tag takes
    // priority
    if (optionalTagTarget.isPresent()) {
      double angleToTarget = Math.toRadians(-optionalTagTarget.get().getYaw());
      targetOrientation = MathUtil.angleModulus(targetOrientation + angleToTarget);
      profiledPIDController.setPID(
          KP_APRIL_TAG.getValue(), KI_APRIL_TAG.getValue(), KD_APRIL_TAG.getValue());
      rSpeed =
          Math.abs(ySpeed) * profiledPIDController.calculate(currentOrientation, targetOrientation);
    } else if (optionalNoteTarget.isPresent()) {
      double angleToTarget = Math.toRadians(noteVision.get().getAngleToBestTarget());
      targetOrientation = MathUtil.angleModulus(targetOrientation + angleToTarget);
      profiledPIDController.setPID(KP_NOTE.getValue(), KI_NOTE.getValue(), KD_NOTE.getValue());
      rSpeed = profiledPIDController.calculate(currentOrientation, targetOrientation);
    } else {
      rSpeed = -xboxController.getRightX();
      rSpeed = MathUtil.applyDeadband(rSpeed, DEADBAND) * inputScalar;
    }

    drivetrain.drive(xSpeed, ySpeed, rSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
