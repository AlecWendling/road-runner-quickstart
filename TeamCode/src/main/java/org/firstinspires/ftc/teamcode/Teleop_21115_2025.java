package org.firstinspires.ftc.teamcode;



import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensors;
import org.firstinspires.ftc.teamcode.mechanisms.Intakes;
import org.firstinspires.ftc.teamcode.mechanisms.Launchers;
import org.firstinspires.ftc.teamcode.mechanisms.Lifts;

@TeleOp()
public final class Teleop_21115_2025 extends LinearOpMode {


    MecanumDriveTeleop drive = new MecanumDriveTeleop();
    ColorSensors colorSensors = new ColorSensors();
    Intakes intakes = new Intakes();
    Launchers launchers = new Launchers();
    AprilTag aprilTag = new AprilTag();



    private int MotorSpeed;
    private boolean leftBumperReleased = true;

    private boolean rightBumperReleased = true;







    @Override



    public void runOpMode() throws InterruptedException {

        MotorSpeed = 1000;//2000;
        launchers.init(hardwareMap);
        colorSensors.init(hardwareMap);
        drive.init(hardwareMap);
        intakes.init(hardwareMap);
        aprilTag.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            //telemetry.addData("RearBallColor",colorSensors.getBackDetectedColor(telemetry));

            if(gamepad1.left_bumper)
            {
                if (leftBumperReleased == true)
                {
                    intakes.toggleIntakes();
                    leftBumperReleased = false;
                }
            }
            else
            {
                leftBumperReleased = true;
            }

            if(gamepad1.right_bumper)
            {
                    //intakes.toggleOutakes();
                    intakes.activateOuttakes();
                    rightBumperReleased = false;
            }
            else
            {
                if (rightBumperReleased == false)
                {
                    intakes.deactivateOutakes();
                    rightBumperReleased = true;
                }
            }

            if (gamepad1.a)
            {
                launchers.backLaunch(MotorSpeed);
            }

            if (gamepad1.b)
            {
                launchers.frontLaunch(MotorSpeed);
            }



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

            // gives directions for drive class
            double forward = -gamepad1.left_stick_y;
            double right = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            /* Process joystick driving control */
            drive.drive(forward, right, rotate);

            /* Call launcher loop processing */
            launchers.launcherLoopProcessing(telemetry);

            aprilTag.showAprilTagDetails(telemetry);

            /* Print debug data */
            telemetry.addData("TargetFlyWheelActualSpeed", MotorSpeed);
            telemetry.update();

        }













    }
}
