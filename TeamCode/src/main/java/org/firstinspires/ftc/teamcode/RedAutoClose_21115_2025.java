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
public final class RedAutoClose_21115_2025 extends LinearOpMode {

    private static final double BACKDISTANCE = 160;
    private static final double FRONTDISTANCE = 160;
    private static final double INTAKETIME = 1.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-47, 50, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Intakes intakes = new Intakes();
        Launchers launchers = new Launchers();
        AprilTag aprilTag = new AprilTag();
        ElapsedTime aprilTagTimer = new ElapsedTime();

        aprilTag.init(hardwareMap);
        intakes.init(hardwareMap);
        launchers.init(hardwareMap);

        TrajectoryActionBuilder tabMoveToReadMotif = drive.actionBuilder(beginPose)
                .lineToX(-12,new TranslationalVelConstraint(60));

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
                    //.stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntakeOff());

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


        drive.updatePoseEstimate();
        pose = drive.localizer.getPose();

        TrajectoryActionBuilder tabMoveToCollect = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(12,30),Math.toRadians(90),new TranslationalVelConstraint(60))
                .stopAndAdd(intakes.rrIntake())
                .strafeTo(new Vector2d(12,65))
                .stopAndAdd(intakes.rrIntakeOff())
                .strafeToLinearHeading(new Vector2d(-12,16),Math.toRadians(48),new TranslationalVelConstraint(60));

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
