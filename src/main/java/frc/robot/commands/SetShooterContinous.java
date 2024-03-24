/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Subsystems;
import java.util.Optional;

/**
 * A command that continously sets arm angle and shooter rpms based on speaker april tag.
 *
 * <p>This command is designed to be bound by whileTrue.
 */
public class SetShooterContinous extends Command {
  private final ArmSubsystem arm;
  private final ShooterSubsystem shooter;
  private final Optional<AprilTagSubsystem> aprilTag;

  private record ShooterParams(double distance, double armAngleDegrees, int rpm) {}

  /** A array of shooter params sorted from nearest to farthest. */
  private final ShooterParams[] shooterParams = { // TODO: finish developing interpolation table
    // new ShooterParams(1.0, -11, 2200),
    new ShooterParams(1.21, -20, 2200),
    new ShooterParams(1.6, -8.0, 2600),
    new ShooterParams(2.0, 2.5, 3100),
    new ShooterParams(2.5, 7.2, 3300),
    new ShooterParams(3.0, 12.4, 3500),
    new ShooterParams(3.5, 14.5, 3750),
    new ShooterParams(4.0, 17.2, 3900),
    new ShooterParams(5.0, 20.0, 4200),
    // new ShooterParams(5.0, 24.5, 4700)
  };

  public SetShooterContinous(Subsystems subsystems) {
    this.arm = subsystems.arm;
    this.shooter = subsystems.shooter;
    this.aprilTag = subsystems.aprilTag;
    addRequirements(arm, shooter);
  }

  @Override
  public void initialize() {
    arm.setGoalAngle(arm.getAngle());
    shooter.setGoalRPM(shooterParams[0].rpm);
  }

  @Override
  public void execute() {
    if (aprilTag.isEmpty()) {
      return;
    }

    int tagId = AprilTagSubsystem.getSpeakerCenterAprilTagID();
    double distance = aprilTag.get().getDistanceToTarget(tagId);

    if (distance == 0) { // if target is not detected
      return;
    }

    if (distance > shooterParams[shooterParams.length - 1].distance) {
      return; // if target is greater than farthest point (last index)
    }

    ShooterParams computedShooterParams = computeShooterParams(distance);

    arm.setGoalAngleContinous(Math.toRadians(computedShooterParams.armAngleDegrees));
    shooter.setGoalRPM(computedShooterParams.rpm);
  }

  // We do not want to shut down shooter or arm so nothing in end method
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  private ShooterParams computeShooterParams(double distance) {
    if (distance <= shooterParams[0].distance) {
      return shooterParams[0];
    }
    for (int i = 0; i < shooterParams.length - 1; i++) {
      ShooterParams p1 = shooterParams[i + 1];
      if (distance <= p1.distance) {
        ShooterParams p0 = shooterParams[i];
        double armAngle =
            interpolate(distance, p0.distance, p1.distance, p0.armAngleDegrees, p1.armAngleDegrees);
        double rpm = interpolate(distance, p0.distance, p1.distance, p0.rpm, p1.rpm);
        return new ShooterParams(distance, armAngle, (int) rpm);
      }
    }
    // returns last element if second check for out of bounds in execute fails
    return shooterParams[shooterParams.length - 1];
  }

  // Ref: https://en.wikipedia.org/wiki/Linear_interpolation
  private double interpolate(double x, double x0, double x1, double y0, double y1) {
    return (y0 * (x1 - x) + y1 * (x - x0)) / (x1 - x0);
  }
}
