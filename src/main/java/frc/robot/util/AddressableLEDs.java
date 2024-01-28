// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class AddressableLEDs {
    private AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;

    public AddressableLEDs(int port, int numberOfLEDs) {
        leds = new AddressableLED(port);
        leds.setLength(numberOfLEDs);
        ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
        leds.setData(ledBuffer);
    }

    public AddressableLED getLED() {
        return leds;
    }

    public void start() {
        leds.start();
    }

    public void stop() {
        leds.stop();
    }

    public void setColor(Color8Bit color) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, color.red, color.green, color.blue);
        }
    }

    public void setColor(Color8Bit color, int index) {
        ledBuffer.setRGB(index, color.red, color.green, color.blue);
    }

    public void commitColor() {
        leds.setData(ledBuffer);
    }
}
