// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

    public static final double ROBOT_LENGTH = Units.inchesToMeters(39);

    /**
     * Wheel diameter.
     */
    public static final double WHEEL_DIAMETER_INCHES = 4.0;

    /** Distance robot should be from speaker to score. */
    public static final double SCORING_DISTANCE_FROM_SPEAKER = Units.inchesToMeters(100);

    /** Distance robot should be from amp to score. */
    public static final double SCORING_DISTANCE_FROM_AMP = Units.inchesToMeters(20);
    
    /** 3d transforms that moves the camera to the center of the robot. */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
      new Translation3d(Units.inchesToMeters(-3.5), Units.inchesToMeters(-14.75), Units.inchesToMeters(-23.25)),
      new Rotation3d(0, Math.toRadians(13.3), Math.toRadians(180))
    );

    /**
     * CAN Ids
     */
    public static class CAN {
      public static class SparkMax {
        public static final int SHOOTER_PORT = 16;
        public static final int INTAKE_PORT = 17; // TODO assign actual port
        public static final int ARM_LEFT_PORT = 18; // TODO assign actual port
        public static final int ARM_RIGHT_PORT = 19; // TODO assign actual port
      }
    }

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
