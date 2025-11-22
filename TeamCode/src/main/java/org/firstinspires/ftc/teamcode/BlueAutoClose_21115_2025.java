package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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
public final class BlueAutoClose_21115_2025 extends LinearOpMode {

    /*  COORDINATE HELPER:
        Subtract from X values to move toward goals, Add to X values to move towards audience
        Subtract from Y values to move toward blue goal, Add to Y values to move towards red goal
        Subtract from Heading to turn counter-clockwise, Add to Heading to turn clockwise
     */
    private static final double START_POS_X_0 = -47;
    private static final double START_POS_Y_0 = -50;
    private static final double START_POS_HEADING_0 = 45;

    private static final double COLLECT_POS_X_1 = 26;
    private static final double COLLECT_POS_Y_1 = -30;
    private static final double COLLECTEND_POS_Y_1 = -65;
    private static final double COLLECT_POS_HEADING_1 = -90;

    private static final double LAUNCH_POS_X_1 = -24;
    private static final double LAUNCH_POS_Y_1 = -16;
    private static final double LAUNCH_POS_HEADING_1 = 150;

    private static final double COLLECT_POS_X_2 = 2;
    private static final double COLLECT_POS_Y_2 = -30;
    private static final double COLLECTEND_POS_Y_2 = -60;
    private static final double COLLECT_POS_HEADING_2 = -90;

    private static final double LAUNCH_POS_X_2 = -40;
    private static final double LAUNCH_POS_Y_2 = -24;
    private static final double LAUNCH_POS_HEADING_2 = 160;

    /* LAUNCHER CONSTANTS */
    private static final double BACKDISTANCE = 160;
    private static final double FRONTDISTANCE = 160;
    private static final double INTAKETIME = 1.25;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(START_POS_X_0, START_POS_Y_0, Math.toRadians(START_POS_HEADING_0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Intakes intakes = new Intakes();
        Launchers launchers = new Launchers();
        AprilTag aprilTag = new AprilTag();
        ElapsedTime aprilTagTimer = new ElapsedTime();

        aprilTag.init(hardwareMap);
        intakes.init(hardwareMap);
        launchers.init(hardwareMap);

        TrajectoryActionBuilder tabMoveToReadMotif = drive.actionBuilder(beginPose)
                .lineToX(-12,new TranslationalVelConstraint(80), new ProfileAccelConstraint(-30,80));

        aprilTagTimer.reset();

        while ((aprilTagTimer.milliseconds() < 5000) && (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.UNKNOWN))
        {
            /* Try to read pattern right at init */
            aprilTag.obeliskPattern = aprilTag.getObeliskPattern(telemetry);
        }

        telemetry.addData("Pattern", aprilTag.obeliskPattern);
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                    tabMoveToReadMotif.build(),
                    aprilTag.rrReadObeliskPattern(),
                    launchers.rrSpinupFront(FRONTDISTANCE),
                    launchers.rrSpinupBack(BACKDISTANCE)
                ));

        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        telemetry.addData("Pattern", aprilTag.obeliskPattern);
        telemetry.update();

        if (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.GPP)
        {
            TrajectoryActionBuilder tabTurnAndShootGPP = drive.actionBuilder(pose)
                    .turnTo(Math.toRadians(142))
                    .stopAndAdd(launchers.rrLaunchFront(FRONTDISTANCE))
                    .stopAndAdd(intakes.rrIntake())
                    .stopAndAdd(launchers.rrFinishLifts())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(launchers.rrLaunchFront(FRONTDISTANCE))
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntakeOff());

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootGPP.build()
                    )
            );

        }
        else if  (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.PGP)
        {
            TrajectoryActionBuilder tabTurnAndShootPGP = drive.actionBuilder(pose)
                    .turnTo(Math.toRadians(142))
                    .stopAndAdd( launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntake())
                    .stopAndAdd(launchers.rrFinishLifts())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE))
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
                    .turnTo(Math.toRadians(142))
                    .stopAndAdd( launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(launchers.rrLaunchFront(FRONTDISTANCE))
                    .stopAndAdd(launchers.rrFinishLifts())
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE))
					.stopAndAdd(intakes.rrIntakeOff());

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootPPG.build()
                    )
            );
        }


        drive.updatePoseEstimate();
        pose = drive.localizer.getPose();

        TrajectoryActionBuilder tabMoveToCollect = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(COLLECT_POS_X_1,COLLECT_POS_Y_1),Math.toRadians(COLLECT_POS_HEADING_1),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-30,80))
                .stopAndAdd(intakes.rrIntake())
                .strafeTo(new Vector2d(COLLECT_POS_X_1,COLLECTEND_POS_Y_1))
                //.stopAndAdd(intakes.rrIntakeOff())
                .stopAndAdd(launchers.rrSpinupFront(FRONTDISTANCE))
                .stopAndAdd(launchers.rrSpinupBack(BACKDISTANCE))
                .strafeToLinearHeading(new Vector2d(LAUNCH_POS_X_1,LAUNCH_POS_Y_1),Math.toRadians(LAUNCH_POS_HEADING_1),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-30,80));

        Actions.runBlocking(
                new ParallelAction(
                        tabMoveToCollect.build()
                ));

        drive.updatePoseEstimate();
        pose = drive.localizer.getPose();

        if (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.GPP)
        {
            TrajectoryActionBuilder tabTurnAndShootGPP = drive.actionBuilder(pose)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchFront(FRONTDISTANCE))
                    .stopAndAdd(intakes.rrIntake())
                    .stopAndAdd(launchers.rrFinishLifts())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(launchers.rrLaunchFront(FRONTDISTANCE))
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntakeOff());

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootGPP.build()
                    )
            );

        }
        else if  (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.PGP)
        {
            TrajectoryActionBuilder tabTurnAndShootPGP = drive.actionBuilder(pose)
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd( launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(launchers.rrFinishLifts())
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
                    .stopAndAdd(intakes.rrIntakeOff())
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntake())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(launchers.rrLaunchFront(FRONTDISTANCE))
                    .stopAndAdd(launchers.rrFinishLifts())
                    .stopAndAdd(launchers.rrLaunchBack(BACKDISTANCE))
                    .stopAndAdd(intakes.rrIntakeOff());

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootPPG.build()
                    )
            );
        }

        drive.updatePoseEstimate();
        pose = drive.localizer.getPose();

        TrajectoryActionBuilder tabCollectFinalArtifacts = drive.actionBuilder(pose)
                //.turnTo(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(COLLECT_POS_X_2,COLLECT_POS_Y_2),Math.toRadians(COLLECT_POS_HEADING_2),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-30,80))
                .stopAndAdd(intakes.rrIntake())
                .strafeTo(new Vector2d(COLLECT_POS_X_2,COLLECTEND_POS_Y_2))
                //.strafeToLinearHeading(new Vector2d(-40,-24),Math.toRadians(160),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-30,80));
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(LAUNCH_POS_X_2, LAUNCH_POS_Y_2, Math.toRadians(LAUNCH_POS_HEADING_2)),Math.toRadians(180));

        Actions.runBlocking(
                new SequentialAction(
                        tabCollectFinalArtifacts.build()
                )
        );

        drive.updatePoseEstimate();
        pose = drive.localizer.getPose();

        //have PPG

        if (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.GPP)
        {
            TrajectoryActionBuilder tabTurnAndShootGPP = drive.actionBuilder(pose)
                    .stopAndAdd(launchers.rrLaunchFront(102))
                    .stopAndAdd(launchers.rrLaunchBack(102))
                    .stopAndAdd(launchers.rrFinishLifts())
                    .waitSeconds(INTAKETIME*2)
                    .stopAndAdd(launchers.rrLaunchBack(102))
                    .stopAndAdd(intakes.rrIntakeOff());

            Actions.runBlocking(
                    new SequentialAction(
                            tabTurnAndShootGPP.build()
                    )
            );
        }
        else if  (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.PGP)
        {
            TrajectoryActionBuilder tabTurnAndShootPGP = drive.actionBuilder(pose)
                    .stopAndAdd(launchers.rrLaunchFront(102))
                    .stopAndAdd(launchers.rrFinishLifts())
                    .waitSeconds(INTAKETIME)
                    .stopAndAdd(launchers.rrLaunchFront(102))
                    .waitSeconds(0.75)
                    .stopAndAdd(launchers.rrLaunchBack(102))
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
                    .stopAndAdd(launchers.rrLaunchBack(102))
                    .stopAndAdd(launchers.rrLaunchFront(102))
                    .waitSeconds(INTAKETIME*2)
                    .stopAndAdd(launchers.rrLaunchBack(102))
                    .stopAndAdd(intakes.rrIntakeOff());

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
