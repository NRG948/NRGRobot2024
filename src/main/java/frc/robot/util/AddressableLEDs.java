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
    public static final int GAMMA_TABLE[] = {
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
        1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
        2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
        5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
       10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
       17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
       25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
       37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
       51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
       69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
       90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
      115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
      144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
      177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
      215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

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
            setColor(color, i + firstLED);
        }
    }

    /**
     * Sets the color at the given LED index.
     * 
     * @param color The color it fills the LEDs.
     * @param index Where the LED is set.
     */
    public void setColor(Color8Bit color, int index) {
        ledBuffer.setRGB(index, GAMMA_TABLE[color.red], GAMMA_TABLE[color.green], GAMMA_TABLE[color.blue]);
    }

    /**
     * Sends the LED data to the LED string.
     */
    public void commitColor() {
        leds.setData(ledBuffer);
    }
}
