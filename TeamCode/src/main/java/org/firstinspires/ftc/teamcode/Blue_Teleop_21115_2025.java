package org.firstinspires.ftc.teamcode;



import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensors;
import org.firstinspires.ftc.teamcode.mechanisms.Intakes;
import org.firstinspires.ftc.teamcode.mechanisms.Launchers;

@Config
@TeleOp()
public final class Blue_Teleop_21115_2025 extends LinearOpMode {


    MecanumDriveTeleop drive = new MecanumDriveTeleop();
    ColorSensors colorSensors = new ColorSensors();
    Intakes intakes = new Intakes();
    Launchers launchers = new Launchers();
    AprilTag aprilTag = new AprilTag();

    FtcDashboard dashboard;
    TelemetryPacket packet;

    /* Adjust this to add offsets to apriltag turn alignment */
    public static double TURNSQUAREOFFSET = 7.6;
    public static double TURN_P = 0.015;
    public static double TURN_I = 0.0005;
    public static double TURN_D = 1.2;


    private int MotorSpeed;
    private boolean leftBumperReleased = true;

    private boolean rightBumperReleased = true;

    private Pose2d posePinpointRobot = new Pose2d(0, 0, 0);





    @Override



    public void runOpMode() throws InterruptedException {

        MotorSpeed = 1600;//2000;
        launchers.init(hardwareMap);
        colorSensors.init(hardwareMap);
        drive.init(hardwareMap,posePinpointRobot);
        intakes.init(hardwareMap);
        aprilTag.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        waitForStart();
        while (opModeIsActive()) {

            //telemetry.addData("RearBallColor",colorSensors.getBackDetectedColor(telemetry));
            //telemetry.addData("FrontBallColor",colorSensors.getFrontDetectedColor(telemetry));

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
                double distance = aprilTag.getDistanceFromBlue(telemetry);
                if (distance != 0)
                {
                    launchers.DistanceLaunchGreen(distance, colorSensors, telemetry);
                }
                else
                {
                    //launchers.LaunchGreen(MotorSpeed, colorSensors,telemetry);
                    launchers.DistanceLaunchGreen(175, colorSensors, telemetry);
                }
            }
            if (gamepad1.b)
            {
                double distance = aprilTag.getDistanceFromBlue(telemetry);
                if (distance != 0)
                {
                    launchers.DistanceLaunchPurple(distance, colorSensors, telemetry);
                }
                else
                {
                    //launchers.LaunchPurple(MotorSpeed, colorSensors,telemetry);
                    launchers.DistanceLaunchPurple(175, colorSensors, telemetry);
                }
            }
            if (gamepad1.xWasPressed())
            {
                double distance = aprilTag.getDistanceFromBlue(telemetry);
                if (distance != 0)
                {
                    launchers.backDistanceLaunch(distance, telemetry);
                }
                else
                {
                    launchers.backDistanceLaunch(175, telemetry);
                    //launchers.backLaunch(MotorSpeed);
                }
            }
            if (gamepad1.yWasPressed())
            {
                double distance = aprilTag.getDistanceFromBlue(telemetry);
                if (distance != 0)
                {
                    launchers.frontDistanceLaunch(distance, telemetry);
                }
                else
                {
                    launchers.frontDistanceLaunch(175, telemetry);
                    //launchers.frontLaunch(MotorSpeed);
                }
            }

            if (gamepad1.dpad_up)
            {
                //drive.turnPID(TARGET_ANG, TURN_P, TURN_I, TURN_D);
                if ((aprilTag.getAngleFromBlue(telemetry) !=999) && (drive.isTurnOverride() == false))
                {
                    drive.turnPID(-(aprilTag.getAngleFromBlue(telemetry)+TURNSQUAREOFFSET), TURN_P, TURN_I, TURN_D);
                }
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
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            colorSensors.updateBallStatus(telemetry);
            /* Process joystick driving control */
            drive.drive(forward, right, rotate, telemetry);
            //drive.driveFieldRelative(forward, right, rotate, telemetry);

            /* Call launcher loop processing */
            launchers.launcherLoopProcessing(telemetry);

            packet.put("AngleError", drive.getPIDAngleError());
            packet.put("RobotAngle", drive.getCurrentAngle());
            packet.put("TargetAngle", drive.getTurnTargetAngle());
            dashboard.sendTelemetryPacket(packet);

            //aprilTag.showAprilTagDetails(telemetry);
            telemetry.addData("AngleFromBlue", aprilTag.getAngleFromBlue(telemetry));
            telemetry.addData("DistanceFromBlue", aprilTag.getDistanceFromBlue(telemetry));
            telemetry.addData("FrontLauncher", colorSensors.frontLauncherStatus);
            telemetry.addData("BackLauncher", colorSensors.backLauncherStatus);
            /* Print debug data */
            //telemetry.addData("ManualTargetFlyWheelSpeed", MotorSpeed);
            telemetry.update();

        }

    }
}
