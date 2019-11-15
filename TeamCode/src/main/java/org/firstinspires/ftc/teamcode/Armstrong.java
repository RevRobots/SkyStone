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

    String clawPos = "Closed";
    //Variables

    public Armstrong (Gamepad g1, Gamepad g2, DcMotor t, DcMotor l, DcMotor a, Servo c)
    {

        gamepad1 = g1;
        gamepad2 = g2;

        turret = t;

        lift = l;

        arm = a;

        claw = c;
        //Constructor

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

            claw.setPosition(0);

        } else if (gamepad2.left_bumper)
        {

            claw.setPosition(1);

        }

    }

    int getRSpd ()
    {

        return this.rSpd;

    }

    double getASpd ()
    {

        return this.aSpd;

    }

    String getClawPos ()
    {

        if (claw.getPosition() == 0)
        {

            clawPos = "Closed";

        } else if (claw.getPosition() == 1)
        {

            clawPos = "Open";

        }

        return this.clawPos;

    }

    void tRight (double spd, int tic)
    {

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setTargetPosition(-tic);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setPower(spd);

        while (turret.isBusy());

        kill();

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    void tLeft (double spd, int tic)
    {

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setTargetPosition(tic);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setPower(spd);

        while (turret.isBusy());

        kill();

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    void up (double spd, int time) throws InterruptedException
    {

        lift.setPower(-spd);

        Thread.sleep(time);

        kill();

    }

    void down (double spd, int time) throws InterruptedException
    {

        lift.setPower(spd);

        Thread.sleep(time);

        kill();

    }

    void rUp (double spd, int tic)
    {

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setTargetPosition(tic);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(spd);

        while (arm.isBusy());

        kill();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    void rDown (double spd, int tic)
    {

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setTargetPosition(-tic);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(spd);

        while (arm.isBusy());

        kill();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    void clamp ()
    {

        claw.setPosition(0);

    }

    void unclamp ()
    {

        claw.setPosition(1);

    }

    void kill ()
    {

        turret.setPower(0);

        lift.setPower(0);

        arm.setPower(0);

        claw.setPosition(0);

    }

}
