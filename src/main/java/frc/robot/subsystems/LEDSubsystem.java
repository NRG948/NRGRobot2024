/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LEDSegment;

/** A subsystem to manage a segment of the total LED string. */
public abstract class LEDSubsystem extends SubsystemBase {
  protected final LEDSegment leds;

  /**
   * Constructs an instance of this class.
   *
   * @param firstLED The index of the first LED in the segment.
   * @param ledCount The number of LEDs in the segment.
   */
  public LEDSubsystem(int firstLED, int ledCount) {
    leds = new LEDSegment(firstLED, ledCount);
  }

  /**
   * Sets the color of the LED at the specified index.
   *
   * @param color The color to set.
   * @param index The index of the LED to set.
   */
  public void setColor(Color8Bit color, int index) {
    leds.setColor(color, index);
  }

  /** Sends the LED data to the LED string. */
  public void commitColor() {
    leds.commitColor();
  }

  /**
   * Fills the LED segment with the specified color.
   *
   * @param color The color to fill the LEDs with.
   */
  public void fillColor(Color8Bit color) {
    leds.fill(color);
  }

  /**
   * Fills the LED segment with the specified color and commits the color to the LEDs.
   *
   * @param color The color to fill the LEDs with.
   */
  public void fillAndCommitColor(Color8Bit color) {
    leds.fill(color);
    commitColor();
  }
}
