package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class DigitalLED {

    LED apriltag_led_red;
    LED apriltag_led_green;

    public void init(HardwareMap hwMap)
    {
        apriltag_led_red = hwMap.get(LED.class, "led_red");
        apriltag_led_green = hwMap.get(LED.class, "led_green");
    }

    public void turnOn()
    {
        apriltag_led_green.off();
        apriltag_led_red.on();
    }

    public void turnOff()
    {
        apriltag_led_green.off();
        apriltag_led_red.off();
    }
}
