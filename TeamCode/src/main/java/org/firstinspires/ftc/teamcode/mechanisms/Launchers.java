package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.mechanisms.Launchers.FlywheelState.RAMPING_UP;
import static org.firstinspires.ftc.teamcode.mechanisms.Lifts.LiftState.IDLE;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launchers {

    public DcMotorEx backFlywheel;

    public DcMotorEx frontFlywheel;

   final double[] backDistancePoints =      {89.99,  90,     137,    213,    260,    300};
   final double[] backSpeedPoints =         {0,      1500,   1600,   1800,   2200,   2500};

    final double[] frontDistancePoints =    {99.99, 100,     107,    142,    198,    300};
    final double[] frontSpeedPoints =       {0,     1900,   1900,   2000,   2200,   2500};

    public enum FlywheelState {
        RAMPING_UP,
        FLYWHEEL_ON,
        FLYWHEEL_OFF
    }
    private static FlywheelState backFlywheelState = FlywheelState.FLYWHEEL_OFF;
    private static FlywheelState frontFlywheelState = FlywheelState.FLYWHEEL_OFF;

    private static ElapsedTime backDelayTimer = new ElapsedTime();
    private static ElapsedTime frontDelayTimer = new ElapsedTime();

    private static double backFlywheelTargetVelocity;
    private static double frontFlywheelTargetVelocity;

    Lifts lifts = new Lifts();

    double backFlywheel_kP = 20.0;
    double backFlywheel_kI = 0.0;
    double backFlywheel_kD = 0.75;
    double backFlywheel_kF = 12;

    double frontFlywheel_kP = 15.0;
    double frontFlywheel_kI = 0.0;
    double frontFlywheel_kD = 0.5;
    double frontFlywheel_kF = 12;

    public void init(HardwareMap hwMap)
    {
        lifts.init(hwMap);

        backFlywheel = hwMap.get(DcMotorEx.class, "BackFlywheel");
        frontFlywheel = hwMap.get(DcMotorEx.class, "FrontFlywheel");

        backFlywheel.setVelocityPIDFCoefficients(backFlywheel_kP, backFlywheel_kI, backFlywheel_kD, backFlywheel_kF);
        frontFlywheel.setVelocityPIDFCoefficients(frontFlywheel_kP, frontFlywheel_kI, frontFlywheel_kD, frontFlywheel_kF);
        backFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void backLaunch(int MotorSpeed)
    {
        backFlywheel.setVelocity(MotorSpeed);
        backFlywheelState = RAMPING_UP;
        backFlywheelTargetVelocity = MotorSpeed;
        backDelayTimer.reset();
    }

    public void frontLaunch(int MotorSpeed)
    {
        frontFlywheel.setVelocity(-MotorSpeed);
        frontFlywheelState = RAMPING_UP;
        frontFlywheelTargetVelocity = -MotorSpeed;
        frontDelayTimer.reset();
    }

    public void LaunchGreen(double motorSpeed, ColorSensors colorSensors, Telemetry telemetry)
    {
        if (colorSensors.backLauncherStatus == ColorSensors.LauncherStatus.GREEN_LOADED)
        {
            backLaunch((int) motorSpeed);
        }
        else if  (colorSensors.frontLauncherStatus == ColorSensors.LauncherStatus.GREEN_LOADED)
        {
            frontLaunch((int) motorSpeed);
        }
    }
    public void LaunchPurple(double motorSpeed, ColorSensors colorSensors, Telemetry telemetry)
    {
        if (colorSensors.backLauncherStatus == ColorSensors.LauncherStatus.PURPLE_LOADED)
        {
            backLaunch((int) motorSpeed);
        }
        else if  (colorSensors.frontLauncherStatus == ColorSensors.LauncherStatus.PURPLE_LOADED)
        {
            frontLaunch((int) motorSpeed);
        }
    }

    public void backDistanceLaunch(double distanceFromTarget, Telemetry telemetry)
    {
        double motorSpeed = interpolate(backDistancePoints, backSpeedPoints, distanceFromTarget);

        //telemetry.addData("interpSpeed", motorSpeed);
        backLaunch((int) motorSpeed);
    }

    public void frontDistanceLaunch(double distanceFromTarget, Telemetry telemetry)
    {
        double motorSpeed = interpolate(frontDistancePoints, frontSpeedPoints, distanceFromTarget);

        //telemetry.addData("interpSpeed", motorSpeed);
        frontLaunch((int) motorSpeed);
    }

    public void DistanceLaunchGreen(double distanceFromTarget, ColorSensors colorSensors, Telemetry telemetry)
    {
        if (colorSensors.backLauncherStatus == ColorSensors.LauncherStatus.GREEN_LOADED)
        {
            backDistanceLaunch(distanceFromTarget,telemetry);
        }
        else if  (colorSensors.frontLauncherStatus == ColorSensors.LauncherStatus.GREEN_LOADED)
        {
            frontDistanceLaunch(distanceFromTarget,telemetry);
        }
    }
    public void DistanceLaunchPurple(double distanceFromTarget, ColorSensors colorSensors, Telemetry telemetry)
    {
        if (colorSensors.backLauncherStatus == ColorSensors.LauncherStatus.PURPLE_LOADED)
        {
            backDistanceLaunch(distanceFromTarget,telemetry);
        }
        else if  (colorSensors.frontLauncherStatus == ColorSensors.LauncherStatus.PURPLE_LOADED)
        {
            frontDistanceLaunch(distanceFromTarget,telemetry);
        }
    }

    public void launcherLoopProcessing(Telemetry telemetry)
    {

        if ((backFlywheelState == RAMPING_UP) && ((backDelayTimer.milliseconds()>=2000) || (abs(backFlywheel.getVelocity())>=abs(backFlywheelTargetVelocity))))
        {
            backFlywheelState = FlywheelState.FLYWHEEL_ON;
            lifts.activateBackLift();
        }
        else if ((backFlywheelState == FlywheelState.FLYWHEEL_ON) && (lifts.getBackLiftState() == IDLE))
        {
            backFlywheel.setVelocity(0);
            backFlywheelState = FlywheelState.FLYWHEEL_OFF;
        }

        if ((frontFlywheelState == RAMPING_UP) && ((frontDelayTimer.milliseconds()>=2500) || (abs(frontFlywheel.getVelocity())>=abs(frontFlywheelTargetVelocity))))
        {
            frontFlywheelState = FlywheelState.FLYWHEEL_ON;
            lifts.activateFrontLift();
        }
        else if ((frontFlywheelState == FlywheelState.FLYWHEEL_ON) && (lifts.getFrontLiftState() == IDLE))
        {
            frontFlywheel.setVelocity(0);
            frontFlywheelState = FlywheelState.FLYWHEEL_OFF;
        }

        lifts.liftLoopProcessing(telemetry);

        //telemetry.addData("BackFlyWheelActualSpeed", backFlywheel.getVelocity());
        //telemetry.addData("FrontFlyWheelActualSpeed", -frontFlywheel.getVelocity());

    }
    // Function to perform linear interpolation for a single target value
    private static double interpolate(double[] distancePoints, double[] speedPoints, double distanceTarget) {
        if (distancePoints.length != speedPoints.length) {
            throw new IllegalArgumentException("distancePoints and speedPoints arrays must have the same length");
        }

        // Handle bounds: before first distance or after last distance
        if (distanceTarget <= distancePoints[0]) {
            return speedPoints[0];
        } else if (distanceTarget >= distancePoints[distancePoints.length - 1]) {
            return speedPoints[speedPoints.length - 1];
        } else {
            // Find interval [distancePoints[j], distancePoints[j+1]] containing distanceTarget
            for (int j = 0; j < distancePoints.length - 1; j++) {
                if (distanceTarget >= distancePoints[j] && distanceTarget <= distancePoints[j + 1]) {
                    // Linear interpolation formula
                    return speedPoints[j]
                            + ((speedPoints[j + 1] - speedPoints[j])
                            / (distancePoints[j + 1] - distancePoints[j]))
                            * (distanceTarget - distancePoints[j]);
                }
            }
        }
        // Fallback (should never reach here if bounds are handled)
        throw new IllegalArgumentException("distanceTarget is out of bounds of distancePoints array");
    }

    //AUTON
    public class RRLaunchBack implements Action {
        private boolean initialized = false;
        private Telemetry telemetry;
        double distanceFromTarget;

        public RRLaunchBack(double distanceFromTarget)
        {
            this.distanceFromTarget = distanceFromTarget;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!initialized)
            {
                backDistanceLaunch(distanceFromTarget,telemetry);
                initialized=true;
            }

            launcherLoopProcessing(telemetry);

            return (backFlywheelState != FlywheelState.FLYWHEEL_OFF);
        }
    }
    public Action rrLaunchBack(double distanceFromTarget) {
        return new Launchers.RRLaunchBack(distanceFromTarget);
    }

    public class RRLaunchFront implements Action {
        private boolean initialized = false;
        private Telemetry telemetry;
        double distanceFromTarget;

        public RRLaunchFront(double distanceFromTarget)
        {
            this.distanceFromTarget = distanceFromTarget;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!initialized)
            {
                frontDistanceLaunch(distanceFromTarget,telemetry);
                initialized=true;
            }

            launcherLoopProcessing(telemetry);

            return (frontFlywheelState != FlywheelState.FLYWHEEL_OFF);
        }
    }
    public Action rrLaunchFront(double distanceFromTarget) {
        return new Launchers.RRLaunchFront(distanceFromTarget);
    }
    //AUTON
}


