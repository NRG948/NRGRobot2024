/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.subsystems;

import static frc.robot.Constants.ColorConstants.*;
import static frc.robot.Constants.RobotConstants.LEDSegment.STATUS_FIRST_LED;
import static frc.robot.Constants.RobotConstants.LEDSegment.STATUS_LED_COUNT;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AddressableLEDs;

public class AddressableLEDSubsystem extends SubsystemBase {
  private AddressableLEDs leds = new AddressableLEDs(STATUS_FIRST_LED, STATUS_LED_COUNT);
  private boolean toggle = false;
  private Joystick joyButton5;
  private Joystick joyButton4;
  private Joystick joyButton3;

  /** Creates a new AddressableLEDSubsystem. */
  public AddressableLEDSubsystem() {
    leds.fill(RED);
    leds.commitColor();
  }

  public void port12() {
    if (joyButton5.getTopPressed()) {
      if (toggle) {
        toggle = false;
      } else {
        toggle = true;
      }
    }
    if (toggle) {
      boolean on = true;
      int x = 0;
      while (on) {
        if (x == 0) {
          leds.fill(BLUE);
          x = 1;
        } else {
          leds.fill(LIGHTBLUE);
          x = 0;
        }
        if (!toggle) {
          on = false;
        }
      }
    }
  }

  public void port11() {
    if (joyButton4.getTopPressed()) {
      if (!toggle) {
        toggle = true;
        leds.fill(YELLOW);
      } else {
        toggle = false;
      }
    }
  }

  public void port10() {
    if (joyButton3.getTopPressed()) {
      if (!toggle) {
        toggle = true;
        leds.fill(RED);
      } else {
        toggle = false;
      }
    }
  }

  public void setColor(Color8Bit color, int index) {
    leds.setColor(color, index);
  }

  public void commitColor() {
    leds.commitColor();
  }

  public void fillColor(Color8Bit color) {
    leds.fill(color);
  }

  public void fillAndCommitColor(Color8Bit color) {
    leds.fill(color);
    commitColor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
