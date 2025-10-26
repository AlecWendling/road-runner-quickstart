package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.mechanisms.Intakes.IntakeState.Intaking;
import static org.firstinspires.ftc.teamcode.mechanisms.Intakes.IntakeState.Idle;
import static org.firstinspires.ftc.teamcode.mechanisms.Intakes.IntakeState.Outtaking;

public class Intakes {

    public CRServo LeftIntake;
    public CRServo RightIntake;

    public CRServo TopIntake;



    public enum IntakeState {

        Outtaking,
        Intaking,

        Idle;


    }
    private IntakeState intakeState;

    public void init(HardwareMap HwMap) {

        LeftIntake = HwMap.get(CRServo.class, "LeftIntake");
        RightIntake = HwMap.get(CRServo.class, "RightIntake");
        TopIntake = HwMap.get(CRServo.class, "TopIntake");
    }
    public void activateIntakes()
    {
        LeftIntake.setPower(-1);
        RightIntake.setPower(1);
        TopIntake.setPower(-1);
        intakeState = Intaking;
    }

    public void deactivateIntakes()
    {
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
        TopIntake.setPower(0);
        intakeState = Idle;
    }

    public void activateOuttakes()
    {
        LeftIntake.setPower(0.5);
        RightIntake.setPower(-0.5);
        TopIntake.setPower(0);
        intakeState = Outtaking;
    }

    public void deactivateOutakes()
    {
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
        TopIntake.setPower(0);
        intakeState = Idle;
    }

    public void toggleIntakes()
    {
        if(intakeState == Idle)
        {
            activateIntakes();
        }
        else
        {
            deactivateIntakes();
        }
    }

    public void toggleOutakes()
    {
        if(intakeState == Idle)
        {
            activateOuttakes();
        }
        else
        {
            deactivateOutakes();
        }
    }

}
