package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="Right Hand Man", group="drive")

public class Drive extends OpMode
{

    Gamepad g1;
    Gamepad g2;

    HardwareMap map;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    DcMotor turret;

    DcMotor lift;

    DcMotor arm;

    Servo claw;

    DriveTrain dT;
    Armstrong a;

    double aTick;

    @Override
    public void init()
    {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        turret = hardwareMap.dcMotor.get("turret");

        lift = hardwareMap.dcMotor.get("lift");

        arm = hardwareMap.dcMotor.get("arm");

        claw = hardwareMap.servo.get("claw");

        dT = new DriveTrain(gamepad1, gamepad2 ,leftFront, rightFront, leftBack, rightBack);
        a = new Armstrong(gamepad1, gamepad2, turret, lift, arm, claw);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        telemetry.addData("Drive Speed: ", dT.getDSpd());
        telemetry.addData("Brake: ", dT.getToggle());

        telemetry.update();

    }

    @Override
    public void stop()
    {

        dT.kill();
        a.kill();

    }

}
