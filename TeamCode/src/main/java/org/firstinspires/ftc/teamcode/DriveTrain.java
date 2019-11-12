package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveTrain
{

    Gamepad gamepad1;
    Gamepad gamepad2;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    double dSpd = 2;
    //Variables

    public DriveTrain (Gamepad g1, Gamepad g2, DcMotor lF, DcMotor rF, DcMotor lB, DcMotor rB)
    {

        leftFront = lF;
        rightFront = rF;
        leftBack = lB;
        rightBack = rB;

    }

    void TeleOpPackage ()
    {

        leftFront.setPower(((-(gamepad1.left_stick_y))+(gamepad1.left_stick_x)+(gamepad1.right_stick_x))/dSpd);
        rightFront.setPower(((gamepad1.left_stick_y)-(gamepad1.left_stick_x)+(gamepad1.right_stick_x))/dSpd);
        leftBack.setPower(((-(gamepad1.left_stick_y))-(gamepad1.left_stick_x)+(gamepad1.right_stick_x))/dSpd);
        rightBack.setPower(((gamepad1.left_stick_y)+(gamepad1.left_stick_x)+(gamepad1.right_stick_x))/dSpd);
        //Directions

        if (gamepad1.y)
        {

            dSpd = 1;
            //full speed

        } else if (gamepad1.x)
        {

            dSpd = 4/3;
            //Three Quarters Speed

        } else if (gamepad1.a)
        {

            dSpd = 2;
            //Half Speed

        } else if (gamepad1.b)
        {

            dSpd = 4;
            //Quarter Speed

        }

        if (gamepad1.right_trigger != 0)
        {

            leftFront.setPower(1);
            rightFront.setPower(-1);
            leftBack.setPower(1);
            rightBack.setPower(-1);
            //Straight Forward

        } else if (gamepad1.left_trigger != 0)
        {

            leftFront.setPower(-1);
            rightFront.setPower(1);
            leftBack.setPower(-1);
            rightBack.setPower(1);
            //Straight Backward

        }

        if (gamepad1.left_bumper)
        {

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //Brake

        } else
        {

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //Un-Brake

        }

    }

    void forward ()
    {



    }

    void backwards ()
    {



    }

    void right ()
    {



    }

    void left ()
    {



    }

    void tRight ()
    {



    }

    void tLeft ()
    {



    }

}
