/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot;

import static frc.robot.Constants.ColorConstants.GREEN;
import static frc.robot.Constants.ColorConstants.PINK;
import static frc.robot.Constants.ColorConstants.RED;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveUsingController;
import frc.robot.commands.InterruptAll;
import frc.robot.commands.LEDs;
import frc.robot.commands.NoteCommands;
import frc.robot.commands.Pathfinding;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StatusLEDSubsystem;
import frc.robot.subsystems.Subsystems;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@RobotPreferencesLayout(groupName = "Preferences", column = 0, row = 0, width = 1, height = 1)
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
    ShooterSubsystem shooter = subsystems.shooter;
    IndexerSubsystem indexer = subsystems.indexer;
    StatusLEDSubsystem statusLED = subsystems.statusLED;

    driverController.start().onTrue(DriveCommands.resetOrientation(subsystems));
    driverController.back().onTrue(new InterruptAll(subsystems));
    driverController.a().onTrue(Pathfinding.pathFindToSpeakerFront());
    driverController.b().onTrue(DriveCommands.autoOrientToSpeaker(subsystems));
    driverController.b().onFalse(DriveCommands.disableAutoOrientation(subsystems));
    driverController.x().onTrue(DriveCommands.autoOrientToNote(subsystems));
    driverController.x().onFalse(DriveCommands.disableAutoOrientation(subsystems));
    driverController.y().whileTrue(Pathfinding.pathFindToAmp2());
    driverController.leftBumper().whileTrue(ClimberCommands.manualClimbChain(subsystems));
    driverController.rightBumper().whileTrue(ClimberCommands.manualClimbDownChain(subsystems));

    // operatorController
    //     .start()
    //     .onTrue(
    //         NoteCommands.prepareToShoot(
    //             subsystems,
    //             Autos.SPIKE_SHOT_RPM.getValue(),
    //             Math.toRadians(Autos.SPIKE_SHOT_ANGLE.getValue())));
    operatorController
        .povUp()
        .whileTrue(NoteCommands.shootAtCurrentRPM(subsystems).finallyDo(shooter::disable));
    operatorController.povDown().whileTrue(NoteCommands.outtake(subsystems));
    // operatorController.povRight().whileTrue(NoteCommands.outakeToAmp(subsystems));
    operatorController.povRight().whileTrue(NoteCommands.shoot(subsystems, 800));
    // operatorController.povLeft().onTrue(ArmCommands.seekToAngle(subsystems, Math.toRadians(15)));
    operatorController.back().onTrue(new InterruptAll(subsystems));
    // operatorController.b().whileTrue(new SetShooterContinous(subsystems));
    // operatorController.b().whileTrue(new SetShooterContinous(subsystems));
    operatorController
        .start()
        .onTrue(
            NoteCommands.prepareToShoot(
                subsystems, Autos.SUBWOOFER_SHOT_RPM.getValue(), ArmSubsystem.STOWED_ANGLE));
    // operatorController.x().onTrue(ArmCommands.seekToAmp(subsystems));
    // operatorController.y().onTrue(ArmCommands.seekToTrap(subsystems));
    // operatorController.a().onTrue(ArmCommands.stow(subsystems));
    operatorController.leftBumper().whileTrue(NoteCommands.intakeUntilNoteDetected(subsystems));
    operatorController.rightBumper().onTrue(NoteCommands.intakeAndAutoCenterNote(subsystems));

    Trigger noteDetected = new Trigger(indexer::isNoteBreakingPracticeBeam);
    noteDetected.onTrue(
        Commands.sequence(LEDs.flashColor(statusLED, GREEN), LEDs.fillColor(statusLED, GREEN)));
    noteDetected.onFalse(LEDs.fillColor(statusLED, RED));

    Trigger shooterSpinning =
        new Trigger(() -> shooter.atGoalRPM() && indexer.isNoteBreakingPracticeBeam());
    shooterSpinning.onTrue(
        Commands.sequence(LEDs.flashColor(statusLED, PINK), LEDs.fillColor(statusLED, PINK)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomous.getAutonomousCommand(subsystems);
  }

  public void disabledInit() {
    coastModeTimer.restart();
    subsystems.intake.disable();
    subsystems.indexer.disable();
    subsystems.shooter.disable();
    subsystems.arm.disable();
    subsystems.drivetrain.disableAutoOrientation();
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
    subsystems.drivetrain.disableAutoOrientation();
  }

  public void periodic() {
    subsystems.periodic();
  }

  public void initShuffleboard() {
    ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator");

    autonomous.addShuffleboardLayout(operatorTab);
    ShuffleboardLayout statusLayout =
        operatorTab
            .getLayout("Status", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 1))
            .withPosition(6, 0)
            .withSize(2, 2);
    statusLayout.addBoolean("Note Detected", subsystems.indexer::isNoteBreakingPracticeBeam);

    RobotPreferences.addShuffleBoardTab();

    subsystems.drivetrain.addShuffleboardTab();

    if (subsystems.aprilTag.isPresent()) {
      subsystems.aprilTag.get().addShuffleboardTab();
    }

    if (subsystems.noteVision.isPresent()) {
      subsystems.noteVision.get().addShuffleboardTab();
    }

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
