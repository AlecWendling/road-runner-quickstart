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
                .strafeToLinearHeading(new Vector2d(36,14),Math.toRadians(90),new TranslationalVelConstraint(60));

        aprilTagTimer.reset();

        while ((aprilTagTimer.milliseconds() < 5000) && (aprilTag.obeliskPattern == AprilTag.ObeliskPattern.UNKNOWN))
        {
            /* Try to read patter right at init */
            aprilTag.obeliskPattern = aprilTag.getObeliskPattern(telemetry);
        }

        telemetry.addData("Pattern", aprilTag.obeliskPattern);
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        tabMoveToReadMotif.build(),
                        aprilTag.rrReadObeliskPattern()
                ));

        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        telemetry.addData("Pattern", aprilTag.obeliskPattern);
        telemetry.update();

        while(opModeIsActive()) {

        }

    }
}
