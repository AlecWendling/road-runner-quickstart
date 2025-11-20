package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class DigitalLED {

    LED apriltag_led_red;
    LED apriltag_led_green;
    LED flywheel_led_red;
    LED flywheel_led_green;

    public void init_apriltagled(HardwareMap hwMap)
    {
        apriltag_led_red = hwMap.get(LED.class, "led_red");
        apriltag_led_green = hwMap.get(LED.class, "led_green");
    }

    public void init_flywheelled(HardwareMap hwMap)
    {
        flywheel_led_red = hwMap.get(LED.class, "fw_led_red");
        flywheel_led_green = hwMap.get(LED.class, "fw_led_red");
    }

    public void apriltag_TurnOnRed()
    {
        apriltag_led_green.off();
        apriltag_led_red.on();
    }

    public void apriltag_TurnOffRed()
    {
        apriltag_led_green.off();
        apriltag_led_red.off();
    }

    public void apriltag_TurnOnGreen()
    {
        apriltag_led_green.on();
    }

    public void apriltag_TurnOffGreen()
    {
        apriltag_led_green.off();
    }

    public void flywheel_TurnOnRed()
    {
        flywheel_led_green.off();
        flywheel_led_red.on();
    }

    public void flywheel_TurnOffRed()
    {
        flywheel_led_green.off();
        flywheel_led_red.off();
    }

    public void flywheel_TurnOnGreen()
    {
        flywheel_led_green.on();
    }

    public void flywheel_TurnOffGreen()
    {
        flywheel_led_green.off();
    }
}
