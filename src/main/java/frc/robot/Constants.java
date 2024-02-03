// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;

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

    public static final double INDEXER_MASS = 0.5;

    /**
     * Wheel diameter.
     */
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    
    /**
     * PMW Ports.
     */
    public static class PWMPort {
      public static final int LED = 2;
    }

    public static class LEDSegment {
      public static final int STATUS_FIRST_LED = 0;
      public static final int STATUS_LED_COUNT = 9;
    }

    /**
     * Field of View of Camera in degrees.
     */
    public static final double CAMERA_FOV = 70;

    /** Distance robot should be from speaker to score. */
    public static final double SCORING_DISTANCE_FROM_SPEAKER = Units.inchesToMeters(100);

    /** Distance robot should be from amp to score. */
    public static final double SCORING_DISTANCE_FROM_AMP = Units.inchesToMeters(20);
    
    /** 3d transforms that moves the camera to the center of the robot. */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
      new Translation3d(Units.inchesToMeters(-3.5), Units.inchesToMeters(-14.75), Units.inchesToMeters(-23.25)),
      new Rotation3d(0, Math.toRadians(13.3), Math.toRadians(180))
    );

    public static final Transform3d NOTE_CAMERA_TO_ROBOT = new Transform3d();  //TODO get real constant

    /**
     * Digital I/O port numbers.
     */
    public static class DigitalIO {
      public static final int INDEXER_BEAM_BREAK = 0; //check if beam is applicable
    }

    /**
     * CAN Ids
     */
    public static class CAN {
      public static class SparkMax {
        public static final int SHOOTER_PORT = 59;
        public static final int INTAKE_PORT = 60; // TODO assign actual port
        public static final int ARM_LEFT_PORT = 61; // TODO assign actual port
        public static final int ARM_RIGHT_PORT = 62; // TODO assign actual port
        public static final int INDEXER_PORT = 63; //TODO assign actual port
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

    /**
     * The number of LEDs on the addressable LED string.
     */
    public static final int LED_COUNT = 63;
  }
    public static class ColorConstants {
      public static final Color8Bit BLACK = new Color8Bit(0,0,0);
      public static final Color8Bit WHITE = new Color8Bit(200, 200, 200);
      public static final Color8Bit RED = new Color8Bit(255,0,0);
      public static final Color8Bit ORANGE = new Color8Bit(255,119,0);
      public static final Color8Bit YELLOW = new Color8Bit(255,165,0);
      public static final Color8Bit GREEN = new Color8Bit(0,204,0);
      public static final Color8Bit BLUE = new Color8Bit(0,0,204);
      public static final Color8Bit PURPLE = new Color8Bit(238, 80, 255);
      public static final Color8Bit PINK = new Color8Bit(255,5,100);
      public static final Color8Bit LIGHTBLUE = new Color8Bit(56,197,252);

      public static final Color8Bit COLORS[] = { BLACK, WHITE, RED, ORANGE, YELLOW, GREEN, BLUE, LIGHTBLUE, PURPLE, PINK};

     }
    
  

}
