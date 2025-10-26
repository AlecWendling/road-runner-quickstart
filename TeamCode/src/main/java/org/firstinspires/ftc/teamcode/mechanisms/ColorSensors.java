package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensors {
    NormalizedColorSensor backColorSensor;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public void init(HardwareMap hwMap)
    {
        backColorSensor = hwMap.get(NormalizedColorSensor.class, "back_color_sensor");
        backColorSensor.setGain(20);
    }

    public DetectedColor getBackDetectedColor(Telemetry telemetry)
    {
        NormalizedRGBA colors = backColorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        //telemetry.addData("red", normRed);
        //telemetry.addData("green", normGreen);
        //telemetry.addData("blue", normBlue);

        if ((normRed < 0.1) && (normGreen > 0.15) && (normBlue > 0.15))
        {
            return DetectedColor.GREEN;
        }
        else if ((normRed > 0.1) && (normGreen > 0.1) && (normGreen < 0.3) && (normBlue > 0.2))
        {
            return DetectedColor.PURPLE;
        }
        else
        {
            return DetectedColor.UNKNOWN;
        }
    }
}
