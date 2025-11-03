package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveTeleop {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    public boolean isTurnOverride() {
        return turnOverride;
    }

    public double inPerTick = 120.0/60721.0;

    public PinpointLocalizer localizer;

    private boolean turnOverride;
    private double turnTargetAngle;
    private static ElapsedTime turnTimer = new ElapsedTime();
    public TurnPIDController turnPIDController = new TurnPIDController(0.0, 0.015, 0.0005, 1.2);

    public double getCurrentAngle()
    {
        return localizer.getHeading();
    }

    public double getTurnTargetAngle()
    {
        return turnTargetAngle;
    }


    public void init(HardwareMap hardwareMap, Pose2d pose){
        frontLeftMotor = hardwareMap.dcMotor.get("left_front_drive");
        frontRightMotor = hardwareMap.dcMotor.get("right_front_drive");
        backLeftMotor = hardwareMap.dcMotor.get("left_rear_drive");
        backRightMotor = hardwareMap.dcMotor.get("right_rear_drive");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        localizer = new PinpointLocalizer(hardwareMap, inPerTick, pose);
    }

    public double getPIDAngleError()
    {
        return turnPIDController.getError();
    }
    private void setPowers(double frontLeftPower,double frontRightPower,double backLeftPower,double backRightPower){
        double maxSpeed = 1.0;

        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

        frontLeftPower /= maxSpeed;
        frontRightPower /= maxSpeed;
        backLeftPower /= maxSpeed;
        backRightPower /= maxSpeed;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void drive(double forward, double right, double rotate, Telemetry telemetry) {
        PoseVelocity2d poseVelocity2d;
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;



        if (turnOverride == false)
        {
            setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        }
        else
        {
            turnToPID(turnTargetAngle);
            //Don't allow more than 3 seconds to complete turn
            if (turnTimer.milliseconds()>3000)
            {
                turnOverride = false;
            }
        }
        poseVelocity2d = localizer.update();
        telemetry.addData("KpTerm", turnPIDController.getKpTerm());
        telemetry.addData("KiTerm", turnPIDController.getKiTerm());
        telemetry.addData("KdTerm", turnPIDController.getKdTerm());
        telemetry.addData("TurnError", turnPIDController.getError());
        telemetry.addData("RobotAngle", localizer.getHeading());
    }

    public void driveFieldRelative(double forward, double right, double rotate, Telemetry telemetry)
    {
        double robotAngle = Math.toRadians(localizer.getHeading());
         // convert to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
         // rotate angle
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate, telemetry);
    }

    public void turnPID(double degrees, double p, double i, double d) {
        turnTimer.reset();
        turnOverride = true;
        turnTargetAngle = localizer.getHeading() - degrees;
        turnPIDController.updateTargetAngle(turnTargetAngle);
        turnPIDController.updateTurnPid(p, i, d);
    }

    private void turnToPID(double targetAngle) {
        double motorPower;

        /* Call this to update initial error term */
        motorPower = turnPIDController.update(localizer.getHeading());

        setPowers(-motorPower, motorPower, -motorPower, motorPower);

        // Checking lastSlope to make sure that it's not oscillating when it quits
        //while ((Math.abs(targetAngle - getHeading(AngleUnit.DEGREES)) > 1) || (Math.abs(targetAngle - getHeading(AngleUnit.DEGREES)) <-359) || pid.getLastSlope() > 0.75)
        if ((Math.abs(turnPIDController.getError()) > 0.5) || turnPIDController.getLastSlope() > 0.75)
        {
            /* Keep turning */
        }
        else
        {
            turnOverride = false;
            ////turnTargetAngle = 999;
        }
    }


}
