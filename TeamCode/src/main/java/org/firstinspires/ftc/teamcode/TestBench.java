package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;

@TeleOp()
public final class TestBench extends LinearOpMode {

    AprilTag aprilTag = new AprilTag();

    @Override

    public void runOpMode() throws InterruptedException {

        aprilTag.init(hardwareMap);

        telemetry.addData("Pattern",aprilTag.getObeliskPattern(telemetry));
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            //aprilTag.showAprilTagDetails(telemetry);
           // aprilTag.getDistanceFromRed(telemetry);
            telemetry.addData("Pattern",aprilTag.getObeliskPattern(telemetry));
            telemetry.addData("Red Distance",aprilTag.getDistanceFromRed(telemetry));
            telemetry.addData("Red Angle",aprilTag.getAngleFromRed(telemetry));
            telemetry.addData("Blue Distance",aprilTag.getDistanceFromBlue(telemetry));
            telemetry.addData("Blue Angle",aprilTag.getAngleFromBlue(telemetry));

            //aprilTag.showAprilTagDetails(telemetry);
            telemetry.update();


        }


    }
}
