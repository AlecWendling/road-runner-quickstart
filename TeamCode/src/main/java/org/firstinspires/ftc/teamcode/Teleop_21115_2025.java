package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public final class Teleop_21115_2025 extends LinearOpMode {

    MecanumDriveTeleop drive = new MecanumDriveTeleop();

    public DcMotorEx backFlywheel;
    public CRServo LeftIntake;
    public CRServo RightIntake;

    public CRServo TopIntake;


    @Override



    public void runOpMode() throws InterruptedException {
        LeftIntake  = hardwareMap.get(CRServo.class, "LeftIntake");
        RightIntake  = hardwareMap.get(CRServo.class, "RightIntake");
        TopIntake  = hardwareMap.get(CRServo.class, "TopIntake");

        drive.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {


            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            boolean update = true;

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
            /* Process joystick driving control */
            drive.drive(forward, right, rotate);

            telemetry.addData("forward", forward);
            telemetry.addData("right", right);
            telemetry.addData("rotate", rotate);
            telemetry.update();

        }













    }
}
