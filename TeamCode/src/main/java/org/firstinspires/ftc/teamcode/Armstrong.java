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

    int rSpd = 2;

    double aSpd = 0.5;

    public Armstrong (Gamepad g1, Gamepad g2, DcMotor t, DcMotor l, DcMotor a, Servo c)
    {

        gamepad1 = g1;
        gamepad2 = g2;

        turret = t;

        lift = l;

        arm = a;

        claw = c;

    }

    void teleOpPackage ()
    {

        turret.setPower(-gamepad2.right_stick_x/rSpd);

        if (gamepad2.a)
        {

            rSpd = 2;

        } else if (gamepad2.b)
        {

            rSpd = 4;

        }

        lift.setPower(gamepad2.left_stick_y);

        if (gamepad2.right_trigger != 0)
        {

            arm.setPower(aSpd);

        } else if (gamepad2.left_trigger != 0)
        {

            arm.setPower(-aSpd);

        } else
        {

            arm.setPower(0.1);

        }

        if (gamepad2.x)
        {

            aSpd = 0.5;

        } else if (gamepad2.y)
        {

            aSpd = 0.25;

        }

        if (gamepad2.right_bumper)
        {

            claw.setPosition(1);

        } else if (gamepad2.left_bumper)
        {

            claw.setPosition(0);

        }

        if (gamepad2.dpad_left)
        {



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

    void kill ()
    {

        turret.setPower(0);

        lift.setPower(0);

        arm.setPower(0);

        claw.setPosition(0);

    }

}
