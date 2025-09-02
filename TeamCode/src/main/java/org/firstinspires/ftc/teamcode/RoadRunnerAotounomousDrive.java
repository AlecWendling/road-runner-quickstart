package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous()
public final class RoadRunnerAotounomousDrive extends LinearOpMode {

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        public DcMotor linearSlide;
        public DcMotor armMotor = null;
        public Servo wrist = null;
        public CRServo intake = null;
        private DcMotor leftRear;
        private DcMotor rightRear;
        private DcMotor leftFront;
        private DcMotor rightFront;
        private IMU imu;

    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    /* Wrist constants */
    final double WRIST_FOLDED_OUT = 0.19;
    final double WRIST_FOLDED_IN = 0.55;

    /* Arm constants */
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647 * 117.0 / 60.0;  //Adjustment for new motor
    final double ARM_SCORE_SPECIMEN = 150 * ARM_TICKS_PER_DEGREE;

    private int leftpos;
    private int rightpos;

    /* Arm variables */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;

    @Override

    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);

            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            double armPower=0;

            /* Hardware Mapping */
            imu = hardwareMap.get(IMU.class, "imu");
            intake = hardwareMap.get(CRServo.class, "intake");
            leftRear = hardwareMap.get(DcMotor.class, "left_rear_drive");
            rightRear = hardwareMap.get(DcMotor.class, "right_rear_drive");
            leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
            rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
            wrist  = hardwareMap.get(Servo.class, "wrist");
            armMotor = hardwareMap.get(DcMotor.class, "left_arm");
            linearSlide = hardwareMap.dcMotor.get("linear_slide");

            /* Linear slide configuration */
            linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            /* Arm Motor configuration */
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .lineToX(35)
                            .splineTo(new Vector2d(50, 140), Math.PI / 2)
                            .strafeTo(new Vector2d(-30,140))
                            .strafeTo(new Vector2d(-30,0))
                            .turnTo(0)
                            .strafeTo(new Vector2d(0,0))
                            //.lineToX(20)
                            .build());


    }
}
