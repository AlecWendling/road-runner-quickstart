package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;

@TeleOp()
public final class Teleop_21115_2025 extends LinearOpMode {

    MecanumDriveTeleop drive = new MecanumDriveTeleop();

    public DcMotorEx backFlywheel;

    public DcMotorEx frontFlywheel;
    public CRServo LeftIntake;
    public CRServo RightIntake;

    public Servo backLift;

    public Servo frontLift;

    public CRServo TopIntake;

    private int MotorSpeed;

    private static ElapsedTime DelayTimer = new ElapsedTime();
    private static boolean startBackFlywheel = false;

    @Override



    public void runOpMode() throws InterruptedException {
        backFlywheel = hardwareMap.get(DcMotorEx.class, "BackFlywheel");
        frontFlywheel = hardwareMap.get(DcMotorEx.class, "FrontFlywheel");
        LeftIntake  = hardwareMap.get(CRServo.class, "LeftIntake");
        RightIntake  = hardwareMap.get(CRServo.class, "RightIntake");
        TopIntake  = hardwareMap.get(CRServo.class, "TopIntake");
        backLift = hardwareMap.get(Servo.class, "BackLift");
        frontLift = hardwareMap.get(Servo.class, "FrontLift");

        double kP = 10.0;
        double kI = 0.0;
        double kD = 0.0;
        double kF = 11.7;
        MotorSpeed = 2000;


        drive.init(hardwareMap);
        //sets lift position be before start
        backLift.setPosition(0.54);
        frontLift.setPosition(0.50); // Done By Brayden He coded he is very happy he did something lines 39-40//
        backFlywheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        frontFlywheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        backFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {



            // gives directions for the robot movement
            double forward = -gamepad1.left_stick_y;
            double right = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;


            if(gamepad1.left_bumper)
            {
                LeftIntake.setPower(-1);
                RightIntake.setPower(1);
                TopIntake.setPower(-1);


            }

            else
            {
                LeftIntake.setPower(0);
                RightIntake.setPower(0);
                TopIntake.setPower(0);
            }
                // line 80-90 front launcher
            if (gamepad1.a)
            {
                //set velocity sets the speed of the motor in encoder ticks
                frontFlywheel.setVelocity(-MotorSpeed);


             //sets lift to launch position
                frontLift.setPosition(0.34);
            }
            else
            {
                //turns motor off and sets lift to original position
                frontFlywheel.setVelocity(0);
                frontLift.setPosition(0.50);

            }
            //line 94-106 back launcher
            if (gamepad1.b)
            {
                backFlywheel.setVelocity(MotorSpeed);
                startBackFlywheel = true;
                DelayTimer.reset();
            }
            if ((startBackFlywheel == true) && (DelayTimer.milliseconds()>=2000))/*(backFlywheel.getVelocity()>=MotorSpeed)*/
            {
                backLift.setPosition(0.34);
            }
            if ((startBackFlywheel == true) && (backLift.getPosition()==0.34)&& (DelayTimer.milliseconds()>=4000))
            {
                backLift.setPosition(0.54);
                backFlywheel.setVelocity(0);
                startBackFlywheel = false;
            }
           /* else
            {
                backFlywheel.setVelocity(0);
                backLift.setPosition(0.54);
            }  */
            //if trigger is pushed it will increase motor speed
            if (gamepad1.right_trigger > 0)
            {
                MotorSpeed = MotorSpeed + 1;
            }
            // if trigger is pushed it decreases motor speed
            if (gamepad1.left_trigger > 0)
            {
                MotorSpeed = MotorSpeed - 1;
            }

            if (MotorSpeed<0) {
                MotorSpeed = 0;
            }

            /* Process joystick driving control */
            drive.drive(forward, right, rotate);

            telemetry.addData("FlyWheelTargetSpeed", MotorSpeed);
            telemetry.addData("FlyWheelActualSpeed", backFlywheel.getVelocity());
            telemetry.addData("Timer", DelayTimer.milliseconds());
            telemetry.update();

        }













    }
}
