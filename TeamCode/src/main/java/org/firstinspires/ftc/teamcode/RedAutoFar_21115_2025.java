package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.Intakes;
import org.firstinspires.ftc.teamcode.mechanisms.Launchers;

@Autonomous()
public final class RedAutoFar_21115_2025 extends LinearOpMode {

    private static final double BACKDISTANCE = 170;
    private static final double FRONTDISTANCE = 160;
    private static final double INTAKETIME = 1.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(62, 14, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Intakes intakes = new Intakes();
        Launchers launchers = new Launchers();
        AprilTag aprilTag = new AprilTag();
        ElapsedTime aprilTagTimer = new ElapsedTime();

        aprilTag.init(hardwareMap);
        intakes.init(hardwareMap);
        launchers.init(hardwareMap);

        TrajectoryActionBuilder tabMoveToReadMotif = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(56,14),Math.toRadians(65),new TranslationalVelConstraint(60));

        aprilTagTimer.reset();

        while ((aprilTagTimer.milliseconds() < 2000) && (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.UNKNOWN))
        {
            /* Try to read patter right at init */
            aprilTag.obeliskPattern = aprilTag.getObeliskPattern(telemetry);
        }

        telemetry.addData("Pattern", aprilTag.obeliskPattern);
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                    tabMoveToReadMotif.build(),
                    aprilTag.rrReadObeliskPattern(),
                    aprilTag.rrReadObeliskPattern()
                ));

        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        telemetry.addData("Pattern", aprilTag.obeliskPattern);
        telemetry.update();

        /* Force pattern to only shoot back launchers */
        aprilTag.obeliskPattern = AprilTag.ObeliskPattern.PGP;

        if (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.GPP)
        {
            TrajectoryActionBuilder tabTurnAndShootGPP = drive.actionBuilder(pose)
                    .stopAndAdd( launchers.rrLaunchFront(300))
                    .stopAndAdd(launchers.rrLaunchBack(300))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(INTAKETIME)
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
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(300))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(INTAKETIME)
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
                    .waitSeconds(INTAKETIME)
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
                .strafeToLinearHeading(new Vector2d(36,30),Math.toRadians(90),new TranslationalVelConstraint(60))
                .stopAndAdd(intakes.rrIntake())
                .strafeTo(new Vector2d(36,65))
                .stopAndAdd(intakes.rrIntakeOff())
                .setTangent(Math.toRadians(260))
                .splineToLinearHeading(new Pose2d(-12, 18, Math.toRadians(48)),Math.toRadians(180),new TranslationalVelConstraint(60));

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
                    .stopAndAdd( launchers.rrLaunchFront(FRONTDISTANCE))
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE));

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
                    .stopAndAdd( launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE));

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
                    .stopAndAdd( launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchFront(FRONTDISTANCE))
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE));

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
