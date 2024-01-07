// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Defines constant values for robot components.
   */
  public static class RobotConstants {
    /**
     * The maximum battery voltage.
     */
    public static final double MAX_BATTERY_VOLTAGE = 12.0;

    /**
     * Wheel diameter.
     */
    public static final double WHEEL_DIAMETER_INCHES = 4.0;

 
  /**
   * Defines operator (i.e. driver and manipulator) constants.
   */
  public static class OperatorConstants {

    /**
     * Defines the port numbers of the Xbox controllers.
     */
    public static class XboxControllerPort {

      public static final int DRIVER = 0;
      public static final int MANIPULATOR = 1;

    }
  }
}
}
