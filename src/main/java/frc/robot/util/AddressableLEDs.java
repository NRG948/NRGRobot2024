// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.RobotConstants;

/** Add your docs here. */
public class AddressableLEDs {
    private static final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(RobotConstants.LED_COUNT);
    private static final AddressableLED leds = createAddressableLED(ledBuffer);
    private final int firstLED;
    private final int numberOfLEDs;

    /**
     * Creates and initializes an addressable LED.
     * 
     * @param buffer The addressable LED buffer.
     */
    private static AddressableLED createAddressableLED(AddressableLEDBuffer buffer) { 
        AddressableLED led = new AddressableLED(RobotConstants.PWMPort.LED);
        led.setLength(RobotConstants.LED_COUNT);
        led.setData(buffer);
        led.start();
        return led;
    }

    /**
     * Constructs an instance of this class.
     * 
     * @param firstLED The index of the first LED.
     * @param numberOfLEDs The number of LEDs.
     */
    public AddressableLEDs(int firstLED, int numberOfLEDs) {
        this.firstLED = firstLED;
        this.numberOfLEDs = numberOfLEDs;
    }

    /**
     * Fills the LED segment with a specific color.
     * 
     * @param color The color it fills the LEDs.
     */
    public void fill(Color8Bit color) {
        for (var i = 0; i < numberOfLEDs; i++) {
            ledBuffer.setRGB(i + firstLED, color.red, color.green, color.blue);
        }
    }

    /**
     * Sets the color at the given LED index.
     * 
     * @param color The color it fills the LEDs.
     * @param index Where the LED is set.
     */
    public void setColor(Color8Bit color, int index) {
        ledBuffer.setRGB(index + firstLED, color.red, color.green, color.blue);
    }

    /**
     * Sends the LED data to the LED string.
     */
    public void commitColor() {
        leds.setData(ledBuffer);
    }
}
