package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveTrain
{

    Gamepad gamepad1;
    Gamepad gamepad2;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Servo leftFoundation;
    Servo rightFoundation;

    double dSpd = 2;

    boolean toggle = false;

    String foundation;
    //Variables

    public DriveTrain (Gamepad g1, Gamepad g2, DcMotor lF, DcMotor rF, DcMotor lB, DcMotor rB, Servo lFo, Servo rFo)
    {

        gamepad1 = g1;
        gamepad2 = g2;

        leftFront = lF;
        rightFront = rF;
        leftBack = lB;
        rightBack = rB;

        leftFoundation = lFo;
        rightFoundation = rFo;

        //Constructor

    }

    void teleOpPackage ()
    {

        leftFront.setPower(((gamepad1.left_stick_y)-(gamepad1.left_stick_x)-(gamepad1.right_stick_x))/dSpd);
        rightFront.setPower(((-(gamepad1.left_stick_y))-(gamepad1.left_stick_x)-(gamepad1.right_stick_x))/dSpd);
        leftBack.setPower(((gamepad1.left_stick_y)+(gamepad1.left_stick_x)-(gamepad1.right_stick_x))/dSpd);
        rightBack.setPower(((-(gamepad1.left_stick_y))+(gamepad1.left_stick_x)-(gamepad1.right_stick_x))/dSpd);
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

            leftFoundation.setPosition(1);
            rightFoundation.setPosition(1);

        } else
        {

            leftFoundation.setPosition(0.5);
            rightFoundation.setPosition(0.4);

        }

        if (gamepad1.left_bumper && toggle==false)
        {

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //Brake

        } else if (gamepad1.left_bumper && toggle==true)
        {

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //Un-Brake

        }

    }

    double getDSpd ()
    {

        return this.dSpd;
        //Return dSpd to main program for telemetry

    }

    boolean getToggle ()
    {

        return this.toggle;
        //Return toggle to main program for telemetry

    }

    String getFoundation ()
    {

        if (leftFoundation.getPosition() == 0)
        {

            foundation = "Down";

        } else if (leftFoundation.getPosition() == 0.5)
        {

            foundation = "Up";

        }

        return this.foundation;

    }

    void forward (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(tic);
        rightFront.setTargetPosition(tic);
        leftBack.setTargetPosition(tic);
        rightBack.setTargetPosition(tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        kill();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Encoder Forward

    }

    void forwardNoStop (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(tic);
        rightFront.setTargetPosition(tic);
        leftBack.setTargetPosition(tic);
        rightBack.setTargetPosition(tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

    }

    void backwards (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(-tic);
        rightFront.setTargetPosition(-tic);
        leftBack.setTargetPosition(-tic);
        rightBack.setTargetPosition(-tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        kill();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Encoder Backwards

    }

    void backwardsNoStop (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(-tic);
        rightFront.setTargetPosition(-tic);
        leftBack.setTargetPosition(-tic);
        rightBack.setTargetPosition(-tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

    }

    void right (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(tic);
        rightFront.setTargetPosition(-tic);
        leftBack.setTargetPosition(-tic);
        rightBack.setTargetPosition(tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        kill();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Encoder Strafe Right

    }

    void rightNoStop (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(tic);
        rightFront.setTargetPosition(-tic);
        leftBack.setTargetPosition(-tic);
        rightBack.setTargetPosition(tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

    }

    void left (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(-tic);
        rightFront.setTargetPosition(tic);
        leftBack.setTargetPosition(tic);
        rightBack.setTargetPosition(-tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        kill();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Encoder Strafe Left

    }

    void leftNoStop (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(-tic);
        rightFront.setTargetPosition(tic);
        leftBack.setTargetPosition(tic);
        rightBack.setTargetPosition(-tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

    }

    void tRight (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(tic);
        rightFront.setTargetPosition(-tic);
        leftBack.setTargetPosition(tic);
        rightBack.setTargetPosition(-tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        kill();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Encoder Turn Right

    }

    void tRightNoStop (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(tic);
        rightFront.setTargetPosition(-tic);
        leftBack.setTargetPosition(tic);
        rightBack.setTargetPosition(-tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

    }

    void tLeft (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(-tic);
        rightFront.setTargetPosition(tic);
        leftBack.setTargetPosition(-tic);
        rightBack.setTargetPosition(tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        kill();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Encoder Turn Left

    }

    void tLeftNoStop (double spd, int tic)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(-tic);
        rightFront.setTargetPosition(tic);
        leftBack.setTargetPosition(-tic);
        rightBack.setTargetPosition(tic);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(spd);
        rightFront.setPower(spd);
        leftBack.setPower(spd);
        rightBack.setPower(spd);

    }

    void grab ()
    {

        leftFoundation.setPosition(1);
        rightFoundation.setPosition(1);

    }

    void release ()
    {

        leftFoundation.setPosition(0.5);
        rightFoundation.setPosition(0.4);

    }

    void kill ()
    {

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);


    }

}
