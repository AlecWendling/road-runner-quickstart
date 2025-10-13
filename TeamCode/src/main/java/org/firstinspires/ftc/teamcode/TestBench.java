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

@TeleOp()
public final class TestBench extends LinearOpMode {

    public DcMotorEx backFlywheel;
    public DcMotorEx frontFlywheel;
    //public DcMotor Motor3;
    public CRServo LeftIntake;
    public CRServo RightIntake;
    public Servo backLift;
    public Servo frontLift;

    FtcDashboard dashboard;


    @Override

    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        backFlywheel = hardwareMap.get(DcMotorEx.class, "BackFlywheel");
        frontFlywheel = hardwareMap.get(DcMotorEx.class, "FrontFlywheel");
        backLift = hardwareMap.get(Servo.class, "BackLift");
        frontLift = hardwareMap.get(Servo.class, "FrontLift");
        //Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        LeftIntake  = hardwareMap.get(CRServo.class, "LeftIntake");
        RightIntake  = hardwareMap.get(CRServo.class, "RightIntake");
        telemetry.addData("Test", "Starting");

        double kP = 10.0;
        double kI = 0.0;
        double kD = 0.0;
        double kF = 11.7;

        backLift.setPosition(0.54);
        frontLift.setPosition(0.50);
        backFlywheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        frontFlywheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            backFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            LeftIntake.setPower(-1);
            RightIntake.setPower(1);

            sleep(5000);

            //frontFlywheel.setVelocity(-2000);
            //backFlywheel.setVelocity(2000);


            sleep(2000);
            LeftIntake.setPower(0);
            RightIntake.setPower(0);
            backLift.setPosition(0.34);
            sleep(1500);
            //frontLift.setPosition(0.34);

            sleep(2000);

            backLift.setPosition(0.54);
            frontLift.setPosition(0.50);
                sleep(2000);
            LeftIntake.setPower(-1);
            RightIntake.setPower(1);
            sleep(2000);
            backLift.setPosition(0.34);
            LeftIntake.setPower(0);
            RightIntake.setPower(0);

            sleep(2000);




            telemetry.addData("FrontFlywheelVelocity", frontFlywheel.getVelocity());
            telemetry.addData("BackFlywheelVelocity", backFlywheel.getVelocity());
            telemetry.addData("BackLiftPosition", backLift.getPosition());
            telemetry.update();




        }


    }
}
