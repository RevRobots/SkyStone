package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Armstrong
{
    Gamepad gamepad1;
    Gamepad gamepad2;

    HardwareMap map;

    DcMotor turret;

    DcMotor lift;

    DcMotor arm;

    Servo claw;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    int rSpd = 2;

    float tDeg;

    double aSpd = 0.5;

    String clawPos = "Closed";
    //Variables

    public Armstrong (Gamepad g1, Gamepad g2, DcMotor t, DcMotor l, DcMotor a, Servo c, BNO055IMU i)
    {

        gamepad1 = g1;
        gamepad2 = g2;

        turret = t;

        lift = l;

        arm = a;

        claw = c;

        imu = i;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        tDeg = angles.firstAngle;
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

    void aTRight (double  spd, int angle)
    {

        tDeg = angles.firstAngle;

        while (tDeg > -angle - 5 || tDeg < -angle + 5)
        {

            tDeg = angles.firstAngle;

            turret.setPower(-spd);

        }

        kill();

    }

    void aTLeft ()
    {



    }

    void aTurn (double spd, int angle, double range)
    {

        tDeg = angles.firstAngle;

        if (angle > tDeg)
        {

            while (tDeg < angle - range || tDeg > angle + range)
            {

                turret.setPower(-spd);

                tDeg = angles.firstAngle;

            }

            tDeg = angles.firstAngle;

        } else if (angle < tDeg)
        {

            while (tDeg < angle - range || tDeg > angle + range)
            {

                turret.setPower(spd);

                tDeg = angles.firstAngle;

            }

            kill();

        }


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

        claw.setPosition(1);

    }

}
