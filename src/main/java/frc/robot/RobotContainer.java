// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static frc.robot.Constants.ColorConstants.ORANGE;
import static frc.robot.Constants.ColorConstants.RED;

import java.util.Set;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants.OperatorConstants;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveUsingController;
import frc.robot.commands.InterruptAll;
import frc.robot.commands.LEDs;
import frc.robot.commands.ManualArmController;
import frc.robot.commands.Pathfinding;
import frc.robot.subsystems.Subsystems;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@RobotPreferencesLayout(groupName = "Preferences", column = 0, row = 0, width = 2, height = 1)
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Subsystems m_subsystems = new Subsystems();

  // Robot autonomous must be initialized after the subsystems
  private final RobotAutonomous m_autonomous = new RobotAutonomous(m_subsystems);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.XboxControllerPort.DRIVER);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.XboxControllerPort.MANIPULATOR);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_subsystems.drivetrain.setDefaultCommand(new DriveUsingController(m_subsystems, m_driverController));
    m_subsystems.armSubsystem.setDefaultCommand(new ManualArmController(m_subsystems, m_operatorController));
    
    // Configure the trigger bindings
    configureBindings();

    initShuffleboard();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `exampleMetdhodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.start().onTrue(DriveCommands.resetOrientation(m_subsystems));
    m_driverController.back().onTrue(new InterruptAll(m_subsystems));
    m_driverController.a().onTrue(Pathfinding.pathFindToSpeakerFront());
    m_driverController.y().onTrue(Commands.defer(() -> DriveCommands.driveToAmp(m_subsystems), 
      Set.of(m_subsystems.drivetrain, m_subsystems.aprilTag)));

    m_operatorController.povUp().onTrue(ArmCommands.seekToTrap(m_subsystems));
    m_operatorController.povRight().onTrue(ArmCommands.seekToAmp(m_subsystems));
    m_operatorController.povDown().onTrue(ArmCommands.stow(m_subsystems));
    m_operatorController.povLeft().onTrue(ArmCommands.disableSeek(m_subsystems));


    Trigger noteDetected = new Trigger(m_subsystems.indexerSubsystem::isNoteDetected);
    noteDetected.onTrue(LEDs.fillColor(m_subsystems.addressableLEDSubsystem, ORANGE));
    noteDetected.onFalse(LEDs.fillColor(m_subsystems.addressableLEDSubsystem, RED));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomous.getAutonomousCommand();
  }

  public void periodic(){
    m_subsystems.periodic();
  }

  public void initShuffleboard(){
    ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator");
    
    m_autonomous.addShuffleboardLayout(operatorTab);

    RobotPreferences.addShuffleBoardTab();
    
    m_subsystems.drivetrain.addShuffleboardTab();
    m_subsystems.aprilTag.addShuffleboardTab();
    m_subsystems.noteVision.addShuffleboardTab();
    m_subsystems.armSubsystem.addShuffleBoardTab();
  }
}
