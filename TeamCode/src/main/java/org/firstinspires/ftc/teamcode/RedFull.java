package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "Red Full Autonomous", group = "Auto")

public class RedFull extends LinearOpMode
{

    AutoMovementStates aMS;

    @Override
    public void runOpMode () throws InterruptedException
    {

        aMS = new AutoMovementStates();

        telemetry.addData(">", "ReadyForStart");

        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {

            aMS.run();

        }

    }

}