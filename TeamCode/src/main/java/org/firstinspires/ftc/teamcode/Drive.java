package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="Right Hand Man", group="drive")

public class Drive extends OpMode
{

    //Wheel Motor
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    //Foundation Servo
    Servo leftFoundation;
    Servo rightFoundation;

    //Turret Motor
    DcMotor turret;

    //Lift Motor
    DcMotor lift;

    //Arm Motor
    DcMotor arm;

    //Claw Continuous Rotation Servos
    CRServo leftClaw;
    CRServo rightClaw;

    Servo capstone;

    //Gyroscope
    BNO055IMU imu;

    //Robot Classes
    DriveTrain dT;
    Armstrong a;

    @Override
    public void init()
    {

        //Setting variables to the real life components using the configuration on the phone.

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

        capstone = hardwareMap.servo.get("capstone");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Setting up the classes to run using the variables above
        dT = new DriveTrain(gamepad1, gamepad2 ,leftFront, rightFront, leftBack, rightBack, leftFoundation, rightFoundation);
        a = new Armstrong(gamepad1, gamepad2, turret, lift, arm, leftClaw, rightClaw, capstone, imu);

        //Set the motors to stop and stay still instead of removing all power and coasting
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Stops Robot
        dT.kill();
        a.kill();

        //Adds a ready to start to the phone. Some inside jokes as well ;)
        telemetry.addData("Let's Goooooooo", "Hehe Yung Link");

        //Displays the telemetry
        telemetry.update();

    }

    @Override
    public void start()
    {

        //Stops Robot
        dT.kill();
        a.kill();

    }

    @Override
    public void loop()
    {

        //Implements each teleop methods from the classes
        dT.teleOpPackage();
        a.teleOpPackage();

        //Displays telemetry that the methods are using
        telemetry.addData("Drive Mode: ", dT.getDSpd());

        telemetry.addData("Turret Mode: ", a.getRSpd());
        telemetry.addData("Turret Encoder:", a.getTurTicks());

        telemetry.addData("Arm Speed: ", a.getASpd());

        //Adds telemetry to phone
        telemetry.update();

    }

    @Override
    public void stop()
    {

        //Stops Robot
        dT.kill();
        a.kill();

    }

}
