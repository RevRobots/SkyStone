package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class EncoderTest extends LinearOpMode
{

    DcMotor turret;

    DcMotor leftFront;

    public void runOpMode()
    {

        turret = hardwareMap.get(DcMotor.class, "turret");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive())
        {

            telemetry.addData("Encider Values: ", turret.getTargetPosition());
            telemetry.addData("Wheel :", leftFront.getTargetPosition());
            telemetry.update();

        }

    }

}