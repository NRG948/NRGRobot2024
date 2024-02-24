/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot;

import static frc.robot.Constants.ColorConstants.ORANGE;
import static frc.robot.Constants.ColorConstants.RED;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants.OperatorConstants;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveUsingController;
import frc.robot.commands.InterruptAll;
import frc.robot.commands.LEDs;
import frc.robot.commands.NoteCommands;
import frc.robot.commands.Pathfinding;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Subsystems;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@RobotPreferencesLayout(groupName = "Preferences", column = 0, row = 0, width = 2, height = 1)
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Subsystems subsystems = new Subsystems();

  // Robot autonomous must be initialized after the subsystems
  private final RobotAutonomous autonomous = new RobotAutonomous(subsystems);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.XboxControllerPort.DRIVER);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.XboxControllerPort.MANIPULATOR);

  private Timer coastModeTimer = new Timer();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    subsystems.drivetrain.setDefaultCommand(new DriveUsingController(subsystems, driverController));
    // subsystems.arm.setDefaultCommand(new ManualArmController(subsystems, operatorController));
    // subsystems.intake.setDefaultCommand(new IntakeUsingController(subsystems,
    // operatorController));
    // subsystems.shooter.setDefaultCommand(new ShootUsingController(subsystems.shooter,
    // operatorController));

    // Configure the trigger bindings
    configureBindings();

    initShuffleboard();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `exampleMetdhodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    driverController.start().onTrue(DriveCommands.resetOrientation(subsystems));
    driverController.back().onTrue(new InterruptAll(subsystems));
    driverController.a().onTrue(Pathfinding.pathFindToSpeakerFront());
    driverController.b().whileTrue(Pathfinding.pathFindToAmp());
    driverController.y().whileTrue(Pathfinding.pathFindToAmp2());

    operatorController.back().onTrue(new InterruptAll(subsystems));
    operatorController.povUp().onTrue(ArmCommands.seekToTrap(subsystems));
    operatorController.povRight().onTrue(ArmCommands.seekToAmp(subsystems));
    operatorController.povDown().onTrue(ArmCommands.stow(subsystems));
    operatorController.povLeft().onTrue(ArmCommands.disableSeek(subsystems));
    operatorController.rightBumper().whileTrue(NoteCommands.intakeUntilNoteDetected(subsystems));
    operatorController.leftTrigger().whileTrue(NoteCommands.outtake(subsystems));

    Trigger noteDetected = new Trigger(subsystems.indexer::isNoteDetected);
    noteDetected.onTrue(LEDs.fillColor(subsystems.addressableLED, ORANGE));
    noteDetected.onFalse(LEDs.fillColor(subsystems.addressableLED, RED));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomous.getAutonomousCommand();
  }

  public void disabledInit() {
    coastModeTimer.restart();
    subsystems.intake.disable();
    subsystems.indexer.disable();
    subsystems.shooter.disable();
    subsystems.arm.disable();
  }

  public void disabledPeriodic() {
    if (coastModeTimer.advanceIfElapsed(3)) {
      subsystems.drivetrain.setBrakeMode(false);
      subsystems.indexer.setBrakeMode(false);
      coastModeTimer.stop();
    }
  }

  public void autonomousInit() {
    subsystems.drivetrain.setBrakeMode(true);
    subsystems.indexer.setBrakeMode(true);
  }

  public void teleopInit() {
    subsystems.drivetrain.setBrakeMode(true);
    subsystems.indexer.setBrakeMode(true);
  }

  public void periodic() {
    subsystems.periodic();
  }

  public void initShuffleboard() {
    ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator");

    autonomous.addShuffleboardLayout(operatorTab);

    RobotPreferences.addShuffleBoardTab();

    subsystems.drivetrain.addShuffleboardTab();
    subsystems.aprilTag.addShuffleboardTab();
    subsystems.noteVision.addShuffleboardTab();

    if (ArmSubsystem.ENABLE_TAB.getValue()) {
      ShuffleboardTab armShooterTab = Shuffleboard.getTab("Arm+Shooter");

      subsystems.arm.addShuffleboardLayout(armShooterTab, subsystems);
      subsystems.shooter.addShuffleboardLayout(armShooterTab, subsystems);
    }

    if (IndexerSubsystem.ENABLE_TAB.getValue()) {
      ShuffleboardTab intakeIndexerTab = Shuffleboard.getTab("Indexer+Intake");

      subsystems.intake.addShuffleboardLayout(intakeIndexerTab);
      subsystems.indexer.addShuffleboardLayout(intakeIndexerTab);
    }
  }
}
