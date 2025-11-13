package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.Intakes;
import org.firstinspires.ftc.teamcode.mechanisms.Launchers;

@Autonomous()
public final class BlueAutoFar_21115_2025 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(62, -14, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Intakes intakes = new Intakes();
        Launchers launchers = new Launchers();
        AprilTag aprilTag = new AprilTag();

        aprilTag.init(hardwareMap);
        intakes.init(hardwareMap);
        launchers.init(hardwareMap);

        TrajectoryActionBuilder tabMoveToReadMotif = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(56,-14),Math.toRadians(115));

        /* Try to read patter right at init */
        aprilTag.obeliskPattern = aprilTag.getObeliskPattern(telemetry);

        telemetry.addData("Pattern", aprilTag.obeliskPattern);
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
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
                    .stopAndAdd( launchers.rrLaunchFront(300))
                    .stopAndAdd(launchers.rrLaunchBack(300))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(300));

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootGPP.build()
                    )
            );

        }
        else if  (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.PGP)
        {
            TrajectoryActionBuilder tabTurnAndShootPGP = drive.actionBuilder(pose)
                    .stopAndAdd( launchers.rrLaunchBack(300))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(300))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(300));

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootPGP.build()
                    )
            );
        }
        else
        {
            TrajectoryActionBuilder tabTurnAndShootPPG = drive.actionBuilder(pose)
                    .stopAndAdd( launchers.rrLaunchBack(300))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchFront(300))
                    .stopAndAdd(launchers.rrLaunchBack(300));

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootPPG.build()
                    )
            );
        }


        drive.updatePoseEstimate();
        pose = drive.localizer.getPose();

        TrajectoryActionBuilder tabMoveToCollect = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(36,-30),Math.toRadians(-90))
                .stopAndAdd(intakes.rrIntake())
                .strafeTo(new Vector2d(36,-65))
                .stopAndAdd(intakes.rrIntakeOff())
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(-12, -18, Math.toRadians(132)),Math.toRadians(180));

        Actions.runBlocking(
                new ParallelAction(
                        tabMoveToCollect.build()
                ));

        drive.updatePoseEstimate();
        pose = drive.localizer.getPose();

        if (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.GPP)
        {
            TrajectoryActionBuilder tabTurnAndShootGPP = drive.actionBuilder(pose)
                    .turnTo(Math.toRadians(132))
                    .stopAndAdd( launchers.rrLaunchFront(160))
                    .stopAndAdd(launchers.rrLaunchBack(160))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(160));

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootGPP.build()
                    )
            );

        }
        else if  (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.PGP)
        {
            TrajectoryActionBuilder tabTurnAndShootPGP = drive.actionBuilder(pose)
                    .turnTo(Math.toRadians(132))
                    .stopAndAdd( launchers.rrLaunchBack(160))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(160))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(160));

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootPGP.build()
                    )
            );
        }
        else
        {
            TrajectoryActionBuilder tabTurnAndShootPPG = drive.actionBuilder(pose)
                    .turnTo(Math.toRadians(132))
                    .stopAndAdd( launchers.rrLaunchBack(160))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(1.0)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchFront(160))
                    .stopAndAdd(launchers.rrLaunchBack(160));

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootPPG.build()
                    )
            );
        }

        while(opModeIsActive()) {

        }

    }
}
