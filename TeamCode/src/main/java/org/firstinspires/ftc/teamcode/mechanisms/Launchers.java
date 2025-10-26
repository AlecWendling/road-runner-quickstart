package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.mechanisms.Launchers.FlywheelState.RAMPING_UP;
import static org.firstinspires.ftc.teamcode.mechanisms.Lifts.LiftState.IDLE;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launchers {

    public DcMotorEx backFlywheel;

    public DcMotorEx frontFlywheel;

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

        if ((frontFlywheelState == RAMPING_UP) && ((frontDelayTimer.milliseconds()>=2000) || (abs(frontFlywheel.getVelocity())>=abs(frontFlywheelTargetVelocity))))
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

        telemetry.addData("BackFlyWheelActualSpeed", backFlywheel.getVelocity());
        telemetry.addData("FrontFlyWheelActualSpeed", -frontFlywheel.getVelocity());

    }
}
