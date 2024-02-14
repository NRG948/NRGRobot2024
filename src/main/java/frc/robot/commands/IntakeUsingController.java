// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Subsystems;

@RobotPreferencesLayout(groupName = "Intake", column = 0, row = 1, width = 2, height = 3)
public class IntakeUsingController extends Command {

  private final IntakeSubsystem intake;
  private final CommandXboxController controller;
  
  @RobotPreferencesValue

  private static final double DEADBAND = 0.1;
  private static final double INTAKE_SPEED = 0.04; //TODO settle on final value

  public RobotPreferences.DoubleValue intake_RPM = new RobotPreferences.DoubleValue("Intake", "intake_RPM", 1.0);
  /** Creates a new InakeUsingController. */
  public IntakeUsingController(Subsystems subsystems, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = subsystems.intake; //need to uncomment in Subsystems.java
    this.controller = controller;

    addRequirements(intake);
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
    speed = MathUtil.clamp(speed, 0.5, 0.7);
    double voltage = speed * RobotConstants.MAX_BATTERY_VOLTAGE;
    intake.runMotor(voltage);
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
