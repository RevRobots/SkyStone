package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class Test extends OpMode
{

    DcMotor test;

    String motorName = "leftFront";

    @Override
    public void init()
    {

        test = hardwareMap.dcMotor.get(motorName);

    }

    @Override
    public void start()
    {



    }

    @Override
    public void loop()
    {

        test.setPower(gamepad1.left_stick_y);

    }

    @Override
    public void stop()
    {



    }

}