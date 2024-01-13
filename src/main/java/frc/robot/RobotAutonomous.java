// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.nrg948.autonomous.Autonomous;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystems;

/** 
 * This class creates and manages the user interface operators used to select
 * and configure autonomous routines.
*/
public class RobotAutonomous {
    private final SendableChooser<Command> chooser;

    /**
     * Creates a new RobotAutonomous.
     * 
     * @param subsystems The subsystems container.
     */
    public RobotAutonomous(Subsystems subsystems) {
        this.chooser = Autonomous.getChooser(subsystems, "frc.robot");
    }

    /**
     * Returns the autonomous command selected in the chooser.
     * 
     * @return The autonomous command selected in the chooser.
     */
    public Command getAutonomousCommand(){
        return this.chooser.getSelected();
    }

    /**
     * Adds the autonomous layout to the shuffleboard tab.
     * 
     * @param tab The tab to add the layout.
     */
    public void addShuffleboardLayout(ShuffleboardTab tab) {
        ShuffleboardLayout layout = tab.getLayout("Autonomous", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 2);
        layout.add("Routine", chooser);
    }
}
