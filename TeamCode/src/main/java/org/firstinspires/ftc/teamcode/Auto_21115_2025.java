package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous()
public final class Auto_21115_2025 extends LinearOpMode {


    @Override

    public void runOpMode() throws InterruptedException {

        telemetry.addData("Test", 5);
        telemetry.update();
        waitForStart();

    }
}
