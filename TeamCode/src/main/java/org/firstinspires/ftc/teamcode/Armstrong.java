package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*This class is made by FTC #12535 Revolutionary Robots for our robot Armstrong. The variables and
methods used in this program are made for the upper half of Armstrong. This includes the turret,
lift, arm, and claw.*/

public class Armstrong
{

    //Gamepad Variables
    Gamepad gamepad1;
    Gamepad gamepad2;

    //Turret Motor
    DcMotor turret;

    //Lift Motor
    DcMotor lift;

    //Arm Motor
    DcMotor arm;

    //Claw Continuous Rotation Servos
    CRServo leftClaw;
    CRServo rightClaw;

    //Capstone Servo
    Servo capstone;

    //Gyroscope
    BNO055IMU imu;

    //Classes for Gyroscope
    Orientation angles;
    Acceleration gravity;

    //Rotation Speed Limiter
    int rSpd = 1;

    //Turret Encoder Tracker
    int turTicks;

    //Turret Angle
    float tDeg;

    //Arm Speed Limiter
    double aSpd = 0.5;

    //Lift Idle Power
    double lIdle = 0;
    //Arm Idle Power
    double aIdle = 0;
    //Claw IdlePower
    double cIdle = 0;

    public Armstrong (Gamepad g1, Gamepad g2, DcMotor t, DcMotor l, DcMotor a, CRServo lC, CRServo rC, Servo c, BNO055IMU i)
    {

        //The constructor of the class allows the class to be used on other programs as well as
        //synchronizing the variables from the main program

        gamepad1 = g1;
        gamepad2 = g2;

        turret = t;

        lift = l;

        arm = a;

        leftClaw = lC;
        rightClaw = rC;

        capstone = c;

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

    }

    void teleOpPackage ()
    {

        if(gamepad2.dpad_left)
        {

            tLeftNoStop(0.25, 950);

        } else if (gamepad2.dpad_right)
        {

            tRightNoStop(0.25, 950);

        } else
        {

            if (!turret.isBusy())
            {

                turret.setPower(-gamepad2.right_stick_x/rSpd);

            }

        }

        //Sets the turret to the value of the right stick including the speed limiter
        //turret.setPower(-gamepad2.right_stick_x/rSpd);

        //Turret Speed Controller
        if (gamepad2.a)
        {

            //Full Speed
            rSpd = 1;

        } else if (gamepad2.b)
        {

            //Half Speed
            rSpd = 2;

        }

        //Sets the power of the lift to the left stick y axis
        lift.setPower(gamepad2.left_stick_y);

        //Arm power controller
        if (gamepad2.right_trigger != 0)
        {

            //Right trigger for moving the arm up
            arm.setPower(aSpd);

        } else if (gamepad2.left_trigger != 0)
        {

            //Left trigger for moving the arm down
            arm.setPower(-aSpd);

        } else
        {

            //Idle
            arm.setPower(0.1);

        }

        if (gamepad2.x)
        {

            aSpd = 0.5;

        } else if (gamepad2.y)
        {

            aSpd = 0.75;

        }

        //Capstone Dropper
        if (gamepad1.dpad_up && gamepad2.y)
        {

            //Drop Capstone
            capstone.setPosition(0);

        } else
        {

            //Stow Capstone
            capstone.setPosition(1);

        }

        //Claw Power Controller
        if (gamepad2.right_bumper)
        {

            //Clamp
            leftClaw.setPower(-1);
            rightClaw.setPower(1);

        } else if (gamepad2.left_bumper)
        {

            //Release
            leftClaw.setPower(1);
            rightClaw.setPower(-1);

        } else
        {

            //Idle
            leftClaw.setPower(0);
            rightClaw.setPower(0);

        }

    }

    int getRSpd ()
    {

        //Returns the turret speed limiter to the main program for telemetry
        return this.rSpd;

    }

    double getASpd ()
    {

        //Returns the arm speed limiter
        return this.aSpd;

    }

    int getTurTicks ()
    {

        //Sets variable to turret encoder
        turTicks = turret.getCurrentPosition();

        //Returns the turret encoder values to the main
        return turTicks;

    }

    void tRight (double spd, int tic)
    {

        //Program to turn turret right in autonomous with encoders
        //The rest of the movement methods will be the same, except the NoStop,
        // lift, and claw methods

        //Stops and resets the encoder on the turret
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets motor to run using the encoder
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Sets motor to run to the given tick position
        turret.setTargetPosition(-tic);

        //Sets motors to run to the target position
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets motor power to given speed
        turret.setPower(spd);

        //Waits until encoder hits target position
        while (turret.isBusy());

        //Stops robot
        kill();

        //Resets encoder for next method
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Allows TeleOp joystick movement if needed
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    void tRightNoStop (double spd, int tic)
    {

        //Same program as other methods as the other, but doesn't include the while loop

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setTargetPosition(-tic);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setPower(spd);
        
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*Awaiting
        if(!turret.isBusy())
        {

            turret.setPower(0);

        }
         */

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

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    void tLeftNoStop (double spd, int tic)
    {

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setTargetPosition(tic);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setPower(spd);

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //Lift Method
    void up (double spd, int time) throws InterruptedException
    {

        //Sets motor power to spd
        lift.setPower(-spd);

        //Waits for the given milliseconds
        Thread.sleep(time);

        //Stops Robot
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
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kill();

    }

    void rUpNoStop (double spd, int tic)
    {

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setTargetPosition(tic);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(spd);

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

    void rDownNoStop (double spd, int tic)
    {

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setTargetPosition(-tic);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(spd);

    }

    void clamp (double spd, int time) throws InterruptedException
    {

        leftClaw.setPower(-spd);
        rightClaw.setPower(spd);

        Thread.sleep(time);

        leftClaw.setPower(cIdle);
        rightClaw.setPower(cIdle);

    }

    void unclamp (double spd, int time) throws InterruptedException
    {

        leftClaw.setPower(spd);
        rightClaw.setPower(-spd);

        Thread.sleep(time);

        leftClaw.setPower(0);
        rightClaw.setPower(0);

    }

    void setCIdle (double i)
    {

        //Sets the idle power of the claw to the given i
        cIdle = i;

    }

    void kill ()
    {

        //Stops all parts of the robot

        turret.setPower(0);

        lift.setPower(0);

        arm.setPower(0);

        leftClaw.setPower(-cIdle);
        rightClaw.setPower(cIdle);

        capstone.setPosition(0.1);

    }

}
