package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Autonomous", group = "Auto")
@Disabled
public class RedAuto extends LinearOpMode
{

    Gamepad gamepad1;
    Gamepad gamepad2;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Servo leftFoundation;
    Servo rightFoundation;

    HardwareMap map;

    DcMotor turret;

    DcMotor lift;

    DcMotor arm;

    CRServo leftClaw;
    CRServo rightClaw;

    BNO055IMU imu;

    DriveTrain dT;
    Armstrong a;

    ElapsedTime runtime;

    AutoMovementStates aMS;

    @Override
    public void runOpMode () throws InterruptedException
    {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFoundation = hardwareMap.servo.get("leftFoundation");
        rightFoundation = hardwareMap.servo.get("rightFoundation");

        turret = hardwareMap.dcMotor.get("turret");

        lift = hardwareMap.dcMotor.get("lift");

        arm = hardwareMap.dcMotor.get("arm");

        leftClaw = hardwareMap.crservo.get("leftClaw");
        rightClaw = hardwareMap.crservo.get("rightClaw");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        dT = new DriveTrain(gamepad1, gamepad2 ,leftFront, rightFront, leftBack, rightBack, leftFoundation, rightFoundation);
        a = new Armstrong(gamepad1, gamepad2,  turret, lift, arm, leftClaw, rightClaw, imu);

        runtime = new ElapsedTime();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dT.kill();
        dT.release();
        a.kill();

        aMS = new AutoMovementStates(leftFront, rightFront, leftBack, rightBack, leftFoundation, rightFoundation, turret, lift, arm, leftClaw, rightClaw, dT, a, runtime);

        telemetry.addData(">", "ReadyForStart");

        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {




        }

    }

}