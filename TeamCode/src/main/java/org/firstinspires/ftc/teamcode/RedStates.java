package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RedStates
{

    /*This class is made by FTC #12535 Revolutionary Robots for our robot Armstrong. The variables and
    methods you find here are for the state machines used to control the robot with help from the
    Drive Train and Armstrong Classes.*/

    //Gamepad Variables
    Gamepad gamepad1;
    Gamepad gamepad2;

    //Wheel Motors
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    //Foundation Servos
    Servo leftFoundation;
    Servo rightFoundation;

    //Turret Motor
    DcMotor turret;

    //Lift Motor
    DcMotor lift;

    //Arm Motor
    DcMotor arm;

    //Claw Continuous Rotation Servo
    CRServo leftClaw;
    CRServo rightClaw;

    //Robot Classes
    DriveTrain dT;
    Armstrong a;

    //Timer to perform timed motor control
    ElapsedTime runtime;

    //Enumerations for the entire set of autonomous programs
    private enum AutoStates
    {

        BEGIN, NAVIGATE_TO_STONE, GRAB_STONE, MOVE_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, NAVIGATE_TO_SECOND_STONE, GRAB_SECOND_STONE, MOVE_TO_FOUNDATION_SSECOND, PLACE_SECOND_STONE, PARK

    }

    //Variable to use in the state machines
    AutoStates cOS = AutoStates.BEGIN;

    //String to display telemetry with
    String currentState = "begin";

    //Enumerations for the wheel base options
    private enum DriveStates
    {

        BEGIN, FORWARD, BACKWARD, RIGHT, LEFT, TURN_RIGHT, TURN_LEFT, IDLE

    }

    //Variable to use in the state machines
    DriveStates cDS = DriveStates.BEGIN;

    //Enumerations for the Foundation Servos
    private enum FoundationGrabberStates
    {

        BEGIN, GRAB, RELEASE, IDLE

    }

    //Variable to use inside of the state machines
    FoundationGrabberStates cFGS = FoundationGrabberStates.BEGIN;

    //Enumerations of turret actions
    private enum TurretStates
    {

        BEGIN, TURN_RIGHT, TURN_LEFT, IDLE

    }

    //Variables to use inside of the state machines
    TurretStates cTS = TurretStates.BEGIN;

    //Enumerations of the actions the lift can do
    private enum LiftStates
    {

        BEGIN, RAISE, LOWER, IDLE

    }

    //Variable to use inside of the state machines
    LiftStates cLS = LiftStates.BEGIN;
    LiftStates pLS;

    //Enumerations for the arm options
    private enum ArmStates
    {

        BEGIN, RAISE, LOWER, IDLE

    }

    //Variables to use in the state machines
    ArmStates cAS = ArmStates.BEGIN;

    //Enumerations for the claw actions
    private enum ClawStates
    {

        BEGIN, GRABBING, RELEASING, GRABBED, RELEASED, IDLE

    }

    //Variable to use in the state machines
    ClawStates cCS = ClawStates.BEGIN;

    public RedStates(DcMotor lF, DcMotor rF, DcMotor lB, DcMotor rB, Servo lFo, Servo rFo, DcMotor t, DcMotor l, DcMotor ar, CRServo lC, CRServo rC, DriveTrain dt, Armstrong armstrong, ElapsedTime rt)
    {

        /*Constructor for our class to be used in our auto programs.
         *Assigns the variables that we set up above using the
         *configuration to assign motors to motors and so on.
         */

        leftFront = lF;
        rightFront = rF;
        leftBack = lB;
        rightBack = rB;

        leftFoundation = lFo;
        rightFoundation = rFo;

        turret = t;

        lift = l;

        arm = ar;

        leftClaw = lC;
        rightClaw = rC;

        dT = dt;

        a = armstrong;

        runtime = rt;

    }

    void runFull ()
    {

        //Method to run the state machine

        //Looks at what cOS equals
        switch (cOS)
        {

            //If cOS equals BEGIN
            case BEGIN:

                //Runs method below
                begin();

                //Exits case
                break;

            //If cOS equals NAVIGATE_TO_STONE
            case NAVIGATE_TO_STONE:

                //Runs method below
                navigateToStoneFull();

                //Exits case
                break;

        }

    }

    void runInside ()
    {

        //Work in progress method for alternative programs

        switch (cOS)
        {

            case BEGIN:

                begin();

                break;

            case NAVIGATE_TO_STONE:

                navigateToStoneInside();

                break;

        }

    }

    String getState ()
    {

        //Returns the state to the phone for telemetry
        return currentState;

    }

    void begin ()
    {

        //Sets the state tracker
        currentState = "begin";

        //Sets cOS to run to the next case/state
        cOS = AutoStates.NAVIGATE_TO_STONE;

    }

    void navigateToStoneFull ()
    {

        //Move to the stone "wall" to detect a skystone

        //Checks to see if the states are finished
        if (cDS != DriveStates.IDLE && cLS != LiftStates.IDLE && cAS != ArmStates.IDLE)
        {

            //Looks at what cDS equals
            switch (cDS)
            {

                //If cDS equals BEGIN
                case BEGIN:

                    //Runs method below
                    cDSBegin1();

                    //Exits case
                    break;

                //If cDS equals FORWARD
                case FORWARD:

                    //Runs method below
                    cDSForwardFull();

                    //Exits case
                    break;

                //If cDS equals IDLE
                case IDLE:

                    //Exits case
                    break;

            }

            //The state machines below do the same as the one above

            switch (cLS)
            {

                case BEGIN:

                    cLSBegin();

                    break;

                case LOWER:

                    cLSLower();

                    break;

                case IDLE:

                    break;


            }

            switch (cAS)
            {

                case BEGIN:

                    cASBegin();

                    break;

                case RAISE:

                    cASRaise();

                    break;

                case IDLE:

                    break;

            }

        } else
        {

            //When all states are idle

            //Sets AutoStates to the next state
            cOS = AutoStates.MOVE_TO_FOUNDATION;
            //Telemetry for the phone
            currentState = "finished";

        }

    }

    void navigateToStoneInside ()
    {

        //work in progress method

        currentState = "Stone";

        if (cDS != DriveStates.IDLE && cLS != LiftStates.IDLE && cAS != ArmStates.IDLE)
        {

            switch (cDS)
            {

                case BEGIN:

                    cDSBegin1();

                    break;

                case FORWARD:

                    cDSForwardInside();

                    break;

                case LEFT:

                    cDSLeft();

                    break;

                case IDLE:

                    break;

            }

            switch (cLS)
            {

                case BEGIN:

                    cLSBegin();

                    break;

                case LOWER:

                    cLSLower();

                    break;

                case IDLE:

                    break;


            }

            switch (cAS)
            {

                case BEGIN:

                    cASBegin();

                    break;

                case RAISE:

                    cASRaise();

                    break;

                case IDLE:

                    break;

            }

        } else
        {

            cOS = AutoStates.MOVE_TO_FOUNDATION;
            currentState = "finished";

        }

    }

    void cDSBegin1 ()
    {

        //Sets the robot to move forward
        dT.forwardNoStop(.25, 850);

        //Sets state to FORWARD
        cDS = DriveStates.FORWARD;

    }

    void cDSForwardFull ()
    {

        //Waits for the encoders to reach target position
        if (!leftFront.isBusy() && !rightFront.isBusy() && !leftBack.isBusy() && !rightBack.isBusy())
        {

            //Stops wheelbase
            dT.kill();

            //Sets wheelbase to idle
            cDS = DriveStates.IDLE;

        }

    }

    void cDSForwardInside ()
    {

        //Waits for the encoders to reach target position
        if (!leftFront.isBusy() && !rightFront.isBusy() && !leftBack.isBusy() && !rightBack.isBusy())
        {

            //Stops wheelbase
            dT.kill();

            //Sets the drive train states to LEFT
            cDS = DriveStates.LEFT;

            //Moves wheelbase to the left
            dT.leftNoStop(0.25, 500);

        }

    }

    void cDSLeft ()
    {

        //Waits for the encoders to reach target position
        if (!leftFront.isBusy() && !rightFront.isBusy() && !leftBack.isBusy() && !rightBack.isBusy())
        {

            //Stops wheelbase
            dT.kill();

            //Sets wheel base to idle
            cDS = DriveStates.IDLE;

        }

    }

    void cLSBegin ()
    {

        //Resets Timer
        runtime.reset();

        //Sets power of the lift to 0.75
        lift.setPower(0.75);

        //Sets LiftStates to LOWER
        cLS = LiftStates.LOWER;

    }

    void cLSLower ()
    {

        //Waits for the timer to hit half a second or 500 milliseconds
        if (runtime.milliseconds() >= 500)
        {

            //Stops lift
            lift.setPower(0);

            //Sets lift to idle
            cLS = LiftStates.IDLE;

        }

    }

    void cASBegin ()
    {

        //Sets arm to rotate up
        a.rUpNoStop(0.25, 250);

        //Sets state to RAISE
        cAS = ArmStates.RAISE;

    }

    void cASRaise ()
    {

        //Waits until arm encoder hits target position
        if (!arm.isBusy())
        {

            //Stops arm
            arm.setPower(0);

            //Sets state to idle
            cAS = ArmStates.IDLE;

        }

    }

}