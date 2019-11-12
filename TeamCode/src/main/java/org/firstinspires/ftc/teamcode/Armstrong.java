package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Armstrong
{
    Gamepad gamepad1;
    Gamepad gamepad2;

    DcMotor turret;

    DcMotor lift;

    DcMotor arm;

    Servo claw;

    public Armstrong (Gamepad g1, Gamepad g2, DcMotor t, DcMotor l, DcMotor a, Servo c)
    {

        gamepad1 = g1;
        gamepad2 = g2;

        turret = t;

        lift = l;

        arm = a;

        claw = c;

    }

    void TeleOpPackage ()
    {

        turret.setPower(gamepad2.right_stick_x);
        lift.setPower(gamepad2.left_stick_y);

        if (gamepad2.right_trigger != 0)
        {

            arm.setPower(1);

        } else if (gamepad2.left_trigger != 0)
        {

            arm.setPower(-1);

        }

        if (gamepad2.right_bumper)
        {

            claw.setPosition(1);

        } else if (gamepad2.left_bumper)
        {

            claw.setPosition(0);

        }

    }

    void tRight ()
    {



    }

    void tLeft ()
    {



    }

    void up ()
    {



    }

    void down ()
    {



    }

    void rUp ()
    {



    }

    void rDown ()
    {



    }

    void clamp ()
    {



    }

    void unclamp ()
    {



    }

}
