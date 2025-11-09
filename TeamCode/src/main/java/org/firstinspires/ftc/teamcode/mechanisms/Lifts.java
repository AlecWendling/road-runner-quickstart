package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.mechanisms.Lifts.LiftState.IDLE;
import static org.firstinspires.ftc.teamcode.mechanisms.Lifts.LiftState.LIFTING_DOWN;
import static org.firstinspires.ftc.teamcode.mechanisms.Lifts.LiftState.LIFTING_UP;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lifts {

    public Servo backLift;

    public Servo frontLift;

    public enum LiftState {
        LIFTING_UP,
        LIFTING_DOWN,
        IDLE
    }

    private LiftState backLiftState;
    private LiftState frontLiftState;

    private static ElapsedTime backLiftDelayTimer = new ElapsedTime();
    private static ElapsedTime frontLiftDelayTimer = new ElapsedTime();

    public void init(HardwareMap hwMap)
    {
        backLift = hwMap.get(Servo.class, "BackLift");
        frontLift = hwMap.get(Servo.class, "FrontLift");

        //sets lift position be before start
        backLift.setPosition(0.54);
        frontLift.setPosition(0.50); // Done By Brayden He coded he is very happy he did something lines 39-40//

        backLiftState = IDLE;
        frontLiftState = IDLE;
    }

    public void activateBackLift()
    {
        backLiftState = LIFTING_UP;
        backLift.setPosition(0.34);
        backLiftDelayTimer.reset();
    }

    public void activateFrontLift()
    {
        frontLiftState = LIFTING_UP;
        frontLift.setPosition(0.30);
        frontLiftDelayTimer.reset();
    }

    public void liftLoopProcessing(Telemetry telemetry)
    {
        if ((backLiftState == LIFTING_UP) && (backLiftDelayTimer.milliseconds() >= 500))
        {
                backLift.setPosition(0.54);
                backLiftState = LIFTING_DOWN;
                backLiftDelayTimer.reset();
        }
        else if ((backLiftState == LIFTING_DOWN) && (backLiftDelayTimer.milliseconds() >= 500))
        {
            backLiftState = IDLE;
        }

        if ((frontLiftState == LIFTING_UP) && (frontLiftDelayTimer.milliseconds() >= 500))
        {
            frontLift.setPosition(0.50);
            frontLiftState = LIFTING_DOWN;
            frontLiftDelayTimer.reset();
        }
        else if ((frontLiftState == LIFTING_DOWN) && (frontLiftDelayTimer.milliseconds() >= 500))
        {
            frontLiftState = IDLE;
        }
    }

    public LiftState getBackLiftState()
    {
        return (backLiftState);
    }

    public LiftState getFrontLiftState()
    {
        return (frontLiftState);
    }
}
