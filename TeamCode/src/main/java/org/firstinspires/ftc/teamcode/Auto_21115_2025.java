package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intakes;

@Autonomous()
public final class Auto_21115_2025 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d beginPose = new Pose2d(0, 0, 0);
        Pose2d beginPose = new Pose2d(53.34, 53.11, -Math.PI / 4);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Intakes intakes = new Intakes();

        intakes.init(hardwareMap);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                        //.turnTo(-Math.PI / 4)
                        .lineToX(16)
                        //splineTo(new Vector2d(50, 140), Math.PI / 2)
                        //.strafeTo(new Vector2d(-30,140))
                        //.strafeTo(new Vector2d(-30,0))
                        .turnTo(-Math.PI / 2)

                        //.strafeTo(new Vector2d(0,0))
                        //.lineToX(20)
                        .build());

        Actions.runBlocking(
                new ParallelAction(
                        intakes.rrIntake(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .lineToY(30)
                                .build()
                ));



        while(opModeIsActive()) {
            telemetry.addData("Pose:", drive.localizer.getPose());
            telemetry.update();
        }

    }
}
