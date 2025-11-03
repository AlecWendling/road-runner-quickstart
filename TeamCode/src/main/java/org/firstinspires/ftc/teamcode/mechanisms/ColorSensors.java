package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensors {
    NormalizedColorSensor backColorSensor;
    NormalizedColorSensor frontColorSensor;
    public LauncherStatus frontLauncherStatus;
    public LauncherStatus backLauncherStatus;

    private static ElapsedTime backSensorChangeDelayTimer = new ElapsedTime();
    private static ElapsedTime frontSensorChangeDelayTimer = new ElapsedTime();

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public enum LauncherStatus {
        PURPLE_LOADED,
        GREEN_LOADED,
        NONE_LOADED
    }

    public void init(HardwareMap hwMap)
    {
        backColorSensor = hwMap.get(NormalizedColorSensor.class, "back_color_sensor");
        backColorSensor.setGain(20);
        frontColorSensor = hwMap.get(NormalizedColorSensor.class, "front_color_sensor");
        frontColorSensor.setGain(20);
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
    public DetectedColor getFrontDetectedColor(Telemetry telemetry)
    {
        NormalizedRGBA colors = frontColorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        //telemetry.addData("red", normRed);
        //telemetry.addData("green", normGreen);
        //telemetry.addData("blue", normBlue);

        if ((normRed < 0.15) && (normGreen > 0.2) && (normBlue > 0.15))
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

    public void updateBallStatus(Telemetry telemetry)
    {
        DetectedColor frontLauncherColor;
        DetectedColor backLauncherColor;

        frontLauncherColor = getFrontDetectedColor(telemetry);
        backLauncherColor = getBackDetectedColor(telemetry);

        if (frontLauncherColor != DetectedColor.UNKNOWN)
        {
            frontSensorChangeDelayTimer.reset();
            if (frontLauncherColor == DetectedColor.PURPLE)
            {
                frontLauncherStatus = LauncherStatus.PURPLE_LOADED;
            }
            else
            {
                frontLauncherStatus = LauncherStatus.GREEN_LOADED;
            }
        }
        else
        {
            if (frontSensorChangeDelayTimer.milliseconds() > 1000)
            {
                frontLauncherStatus = LauncherStatus.NONE_LOADED;
            }
        }

        if (backLauncherColor != DetectedColor.UNKNOWN)
        {
            backSensorChangeDelayTimer.reset();
            if (backLauncherColor == DetectedColor.PURPLE)
            {
                backLauncherStatus = LauncherStatus.PURPLE_LOADED;
            }
            else
            {
                backLauncherStatus = LauncherStatus.GREEN_LOADED;
            }
        }
        else
        {
            if (backSensorChangeDelayTimer.milliseconds() > 1000)
            {
                backLauncherStatus = LauncherStatus.NONE_LOADED;
            }
        }
    }
}
