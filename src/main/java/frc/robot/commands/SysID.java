/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.SwerveModuleVelocities;
import frc.robot.util.SwerveModuleVoltages;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import org.javatuples.LabelValue;

/** A utility class to create SysID logging commands. */
public class SysID {
  private static final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  private static final MutableMeasure<Distance> distance = mutable(Meters.of(0));
  private static final MutableMeasure<Velocity<Distance>> velocity = mutable(MetersPerSecond.of(0));
  private static final MutableMeasure<Angle> angle = mutable(Radians.of(0));
  private static final MutableMeasure<Velocity<Angle>> angularVelocity =
      mutable(RadiansPerSecond.of(0));
  private static int currentDriveTest = 0;
  private static int currentSteeringTest = 0;

  /**
   * Returns the commands to characterize the swerve drive.
   *
   * @param subsystems The subsystems container.
   * @return The commands to characterize the swerve drive.
   */
  public static Collection<LabelValue<String, Command>> getSwerveDriveCharacterizationCommands(
      Subsystems subsystems) {
    SysIdRoutine.Config routineConfig = new SysIdRoutine.Config();
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
              SwerveModuleVoltages voltages = new SwerveModuleVoltages(volts.in(Volts), 0.0);
              SwerveModuleVoltages[] moduleVoltages =
                  new SwerveModuleVoltages[] {voltages, voltages, voltages, voltages};
              subsystems.drivetrain.setModuleVoltages(moduleVoltages);
              appliedVoltage.mut_replace(volts);
            },
            (SysIdRoutineLog log) -> {
              log.motor("Drive")
                  .voltage(appliedVoltage)
                  .linearPosition(
                      distance.mut_replace(subsystems.drivetrain.getPosition().getX(), Meters))
                  .linearVelocity(
                      velocity.mut_replace(
                          subsystems.drivetrain.getChassisSpeeds().vxMetersPerSecond,
                          MetersPerSecond));
            },
            subsystems.drivetrain);
    SysIdRoutine routine = new SysIdRoutine(routineConfig, mechanism);
    return List.of(
        new LabelValue<String, Command>(
            "Swerve Drive Quasistatic Forward", routine.quasistatic(Direction.kForward)),
        new LabelValue<String, Command>(
            "Swerve Drive Quasistatic Reverse", routine.quasistatic(Direction.kReverse)),
        new LabelValue<String, Command>(
            "Swerve Drive Dynamic Forward", routine.dynamic(Direction.kForward)),
        new LabelValue<String, Command>(
            "Swerve Drive Dynamic Reverse", routine.dynamic(Direction.kReverse)));
  }

  /**
   * Returns the command sequence used to characterize the swerve drive.
   *
   * @param subsystems The subsystems container.
   * @return The command sequence.
   */
  public static Command getSwerveDriveCharacterizationSequence(Subsystems subsystems) {
    Command[] testCommands =
        getSwerveDriveCharacterizationCommands(subsystems).stream()
            .map(lv -> lv.getValue())
            .toArray(Command[]::new);

    Command testSequence =
        Commands.select(
            Map.of(
                0, testCommands[0],
                1, testCommands[1],
                2, testCommands[2],
                3, testCommands[3]),
            () -> {
              int testIndex = currentDriveTest++;
              if (currentDriveTest >= 4) {
                currentDriveTest = 0;
              }
              return testIndex;
            });
    return testSequence;
  }

  public static Collection<LabelValue<String, Command>> getSwerveSteeringCharacterizationCommands(
      Subsystems subsystems) {
    SysIdRoutine.Config routineConfig = new SysIdRoutine.Config();
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
              SwerveModuleVoltages voltages = new SwerveModuleVoltages(0.0, volts.in(Volts));
              SwerveModuleVoltages[] moduleVoltages =
                  new SwerveModuleVoltages[] {voltages, voltages, voltages, voltages};
              subsystems.drivetrain.setModuleVoltages(moduleVoltages);
              appliedVoltage.mut_replace(volts);
            },
            (SysIdRoutineLog log) -> {
              SwerveModuleState[] state = subsystems.drivetrain.getModuleStates();
              SwerveModuleVelocities[] velocities = subsystems.drivetrain.getModuleVelocities();
              for (int i = 0; i < 4; i++) {
                String moduleName = "Module " + i;
                log.motor(moduleName)
                    .voltage(appliedVoltage)
                    .angularPosition(angle.mut_replace(state[i].angle.getRadians(), Radians))
                    .angularVelocity(
                        angularVelocity.mut_replace(
                            velocities[i].steeringVelocity, RadiansPerSecond));
              }
            },
            subsystems.drivetrain);
    SysIdRoutine routine = new SysIdRoutine(routineConfig, mechanism);
    return List.of(
        new LabelValue<String, Command>(
            "Swerve Steering Quasistatic Forward", routine.quasistatic(Direction.kForward)),
        new LabelValue<String, Command>(
            "Swerve Steering Quasistatic Reverse", routine.quasistatic(Direction.kReverse)),
        new LabelValue<String, Command>(
            "Swerve Steering Dynamic Forward", routine.dynamic(Direction.kForward)),
        new LabelValue<String, Command>(
            "Swerve Steering Dynamic Reverse", routine.dynamic(Direction.kReverse)));
  }

  public static Command getSwerveSteeringCharacterizationSequence(Subsystems subsystems) {
    Command[] testCommands =
        getSwerveSteeringCharacterizationCommands(subsystems).stream()
            .map(lv -> lv.getValue())
            .toArray(Command[]::new);

    Command testSequence =
        Commands.select(
            Map.of(
                0, testCommands[0],
                1, testCommands[1],
                2, testCommands[2],
                3, testCommands[3]),
            () -> {
              int testIndex = currentSteeringTest++;
              if (currentSteeringTest >= 4) {
                currentSteeringTest = 0;
              }
              return testIndex;
            });
    return testSequence;
  }
}
