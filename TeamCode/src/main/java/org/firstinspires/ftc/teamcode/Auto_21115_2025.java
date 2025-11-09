package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.Intakes;
import org.firstinspires.ftc.teamcode.mechanisms.Launchers;

@Autonomous()
public final class Auto_21115_2025 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d beginPose = new Pose2d(0, 0, 0);
        Pose2d beginPose = new Pose2d(-47, 50, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Intakes intakes = new Intakes();
        Launchers launchers = new Launchers();
        AprilTag aprilTag = new AprilTag();

        aprilTag.init(hardwareMap);
        intakes.init(hardwareMap);
        launchers.init(hardwareMap);

        TrajectoryActionBuilder tabMoveToReadMotif = drive.actionBuilder(beginPose)
                .lineToX(-12);



        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                    //intakes.rrIntake(),
                    tabMoveToReadMotif.build(),
                    aprilTag.rrReadObeliskPattern()
                ));

        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        telemetry.addData("Pattern", aprilTag.obeliskPattern);
        telemetry.update();

        if (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.GPP)
        {
            TrajectoryActionBuilder tabTurnAndShootGPP = drive.actionBuilder(pose)
                    .turnTo(Math.toRadians(48))
                    .stopAndAdd( launchers.rrLaunchFront())
                    .stopAndAdd(launchers.rrLaunchBack())
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack());

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootGPP.build()
                    )
            );

        }
        else if  (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.PGP)
        {
            TrajectoryActionBuilder tabTurnAndShootPGP = drive.actionBuilder(pose)
                    .turnTo(Math.toRadians(48))
                    .stopAndAdd( launchers.rrLaunchBack())
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack())
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack());

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootPGP.build()
                    )
            );
        }
        else
        {
            TrajectoryActionBuilder tabTurnAndShootPPG = drive.actionBuilder(pose)
                    .turnTo(Math.toRadians(48))
                    .stopAndAdd( launchers.rrLaunchBack())
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchFront())
                    .stopAndAdd(launchers.rrLaunchBack());

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootPPG.build()
                    )
            );
        }


        drive.updatePoseEstimate();
        pose = drive.localizer.getPose();

        TrajectoryActionBuilder tabMoveToCollect = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(12,30),Math.toRadians(90))
                .stopAndAdd(intakes.rrIntake())
                .strafeTo(new Vector2d(12,65))
                .stopAndAdd(intakes.rrIntakeOff())
                .strafeToLinearHeading(new Vector2d(-12,16),Math.toRadians(48));

        Actions.runBlocking(
                new ParallelAction(
                        tabMoveToCollect.build()
                ));

        drive.updatePoseEstimate();
        pose = drive.localizer.getPose();

        if (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.GPP)
        {
            TrajectoryActionBuilder tabTurnAndShootGPP = drive.actionBuilder(pose)
                    .turnTo(Math.toRadians(48))
                    .stopAndAdd( launchers.rrLaunchFront())
                    .stopAndAdd(launchers.rrLaunchBack())
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack());

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootGPP.build()
                    )
            );

        }
        else if  (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.PGP)
        {
            TrajectoryActionBuilder tabTurnAndShootPGP = drive.actionBuilder(pose)
                    .turnTo(Math.toRadians(48))
                    .stopAndAdd( launchers.rrLaunchBack())
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack())
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack());

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootPGP.build()
                    )
            );
        }
        else
        {
            TrajectoryActionBuilder tabTurnAndShootPPG = drive.actionBuilder(pose)
                    .turnTo(Math.toRadians(48))
                    .stopAndAdd( launchers.rrLaunchBack())
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchFront())
                    .stopAndAdd(launchers.rrLaunchBack());

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootPPG.build()
                    )
            );
        }

        while(opModeIsActive()) {
            //telemetry.addData("Pose:", drive.localizer.getPose());
            //telemetry.update();
            //telemetry.addData("distFromRed", aprilTag.getDistanceFromRed(telemetry));
            //telemetry.update();
        }

    }
}
