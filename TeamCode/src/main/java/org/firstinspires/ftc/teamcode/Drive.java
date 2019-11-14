package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drive extends OpMode
{

    Gamepad g1;
    Gamepad g2;

    HardwareMap map;

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    DcMotor turret = null;

    DcMotor lift = null;

    DcMotor arm = null;

    Servo claw;

    DriveTrain dT;
    Armstrong a;

    @Override
    public void init()
    {

        leftFront = map.dcMotor.get("leftFront");
        rightFront = map.dcMotor.get("rightFront");
        leftBack = map.dcMotor.get("leftBack");
        rightBack = map.dcMotor.get("rightBack");

        turret = map.dcMotor.get("turret");

        lift = map.dcMotor.get("lift");

        arm = map.dcMotor.get("arm");

        claw = map.servo.get("claw");

        dT = new DriveTrain(g1, g2, leftFront, rightFront, leftBack, rightBack);
        a = new Armstrong(g1, g2, turret, lift, arm, claw);

        dT.kill();
        a.kill();

    }

    @Override
    public void start()
    {

        dT.kill();
        a.kill();

    }

    @Override
    public void loop()
    {

        dT.teleOpPackage();
        a.teleOpPackage();

    }

    @Override
    public void stop()
    {

        dT.kill();
        a.kill();

    }

}
