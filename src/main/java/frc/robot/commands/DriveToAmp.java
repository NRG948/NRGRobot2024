// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToAmp extends Command {

  private static final double kP_X = 0.2;
  private static final double kP_Y = 0.01;
  private static final double kP_R = 0.01;
  private static final double STOP_DISTANCE = 1;

  private final AprilTagSubsystem aprilTag;
  private final SwerveSubsystem drivetrain;

  private double xSpeed = 0;
  private double ySpeed = 0;
  private double rSpeed = 0;

  private double distanceToStopPoint;
  private Timer noTargetTimer = new Timer();

  /** Creates a new DriveToAmp. */
  public DriveToAmp(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain;
    this.aprilTag = subsystems.aprilTag;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noTargetTimer.start();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    int id = AprilTagSubsystem.getAmpAprilTagID();
    Optional<PhotonTrackedTarget> optionalTarget = aprilTag.getTarget(id); 
    
    if (optionalTarget.isPresent()) {
      noTargetTimer.reset();
      var bestCameraToTarget = optionalTarget.get().getBestCameraToTarget();
      distanceToStopPoint = Math.hypot(bestCameraToTarget.getX(), bestCameraToTarget.getY()) - STOP_DISTANCE;
      var skewToTarget = optionalTarget.get().getSkew();
      var tagPose = aprilTag.getTagPose2d(id);

      xSpeed = distanceToStopPoint * kP_X;
      xSpeed = clamp(xSpeed, -0.4, 0.4);
      ySpeed = skewToTarget * kP_Y;
      rSpeed = (drivetrain.getOrientation().getDegrees() - tagPose.getRotation().getDegrees()) * kP_R;
    } else {
      if (noTargetTimer.hasElapsed(0.1)){
        xSpeed = 0;
        ySpeed = 0;
        rSpeed = 0;
      }
    } 
    drivetrain.drive(
      -xSpeed,
      ySpeed,
      0, //rSpeed,
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distanceToStopPoint < 0;
  }

  private double clamp(double value, double min, double max){
    if (value > max){
      return max;
    } else if (value < min){
      return min;
    } else{
      return value;
    }
  }
}
