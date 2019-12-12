package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoMovementStates
{

    /*This class is to use state machines to be able to
     *move more than one part of Armstrong, our robot,
     *at once. This controls all of the robot with help
     * of the DriveTrain class and Armstrong Class for
     * robot movement. This should be used inside of the
     * Red Full Autonomous and the Blue Full Autonomous.
     */

    /*Below we set up variables for the rest of our program.
     *We assign controllers, the configuration with HardwareMap,
     *DcMotors, Servos, Continuous Rotation Servo, Gyroscope,
     *and classes.
     */

    Gamepad gamepad1;
    Gamepad gamepad2;

    HardwareMap hardwareMap;

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

    int alternate = 1;

    /*Now we set up our enumerations for our state machine.
     *The enums include the entire auto program, drive train,
     *turret, lift, arm, and claw.
     */

    private enum AutoStates
    {

        BEGIN, NAVIGATE_TO_STONE, GRAB_STONE, MOVE_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, NAVIGATE_TO_SECOND_STONE, GRAB_SECOND_STONE, MOVE_TO_FOUNDATION_SSECOND, PLACE_SECOND_STONE, PARK

    }

    AutoStates cOS;
    AutoStates pOS;

    private enum DriveStates
    {

        BEGIN, FORWARD, BACKWARD, RIGHT, LEFT, TURN_RIGHT, TURN_LEFT, IDOL

    }

    DriveStates cDS;
    DriveStates pDS;

    Boolean isFirstDrive = true;

    Boolean isDriveStateFinished = false;

    private enum FoundationGrabberStates
    {

        BEGIN, GRAB, RELEASE, IDOL

    }

    FoundationGrabberStates cFGS;
    FoundationGrabberStates pFGS;

    Boolean isFirstFoundation = true;

    Boolean isFoundationStateFinished = false;

    private enum TurretStates
    {

        BEGIN, TURN_RIGHT, TURN_LEFT, IDOL

    }

    TurretStates cTS;
    TurretStates pTS;

    Boolean isFirstTurret = true;

    Boolean isTurretStateFinished = false;

    private enum LiftStates
    {

        BEGIN, RAISING, LOWERING, RAISED, LOWERED, IDOL

    }

    LiftStates cLS;
    LiftStates pLS;

    Boolean isFirstLift = true;

    Boolean isLiftStateFinished = false;

    private enum ArmStates
    {

        BEGIN, RAISE, LOWER, RAISED, LOWERED, IDOL

    }

    ArmStates cAS;
    ArmStates pAS;

    Boolean isFirstArm = true;

    Boolean isArmStateFinished = false;

    private enum ClawStates
    {

        BEGIN, GRABBING, RELEASING, GRABBED, RELEASED, IDOL

    }

    ClawStates cCS;
    ClawStates pCS;

    Boolean isFirstClaw = true;

    Boolean isClawStateFinished = false;

    /*Constructor for our class to be used in our auto programs.
     *Assigns the variables that we set up above using the
     *configuration to assign motors to motors and so on.
     */

    public AutoMovementStates ()
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

    }

    public void run()
    {

        switch (cOS)
        {

            case BEGIN:

                switch (cDS)
                {

                    case BEGIN:

                        dT.kill();

                        cDS = DriveStates.IDOL;
                        pDS = DriveStates.BEGIN;

                        break;


                    case IDOL:

                        dT.kill();

                        cDS = DriveStates.RIGHT;
                        pDS = DriveStates.IDOL;

                        isDriveStateFinished = true;

                        break;

                }

                switch (cFGS)
                {

                    case BEGIN:

                        dT.release();

                        cFGS = FoundationGrabberStates.IDOL;
                        pFGS = FoundationGrabberStates.BEGIN;

                        break;

                    case IDOL:

                        dT.release();

                        cFGS = FoundationGrabberStates.IDOL;
                        pFGS = FoundationGrabberStates.IDOL;

                        isFoundationStateFinished = true;

                        break;

                }

                switch (cTS)
                {

                    case BEGIN:

                        turret.setPower(0);

                        cTS = TurretStates.IDOL;
                        pTS = TurretStates.BEGIN;

                        break;

                    case IDOL:

                        turret.setPower(0);

                        cTS = TurretStates.TURN_RIGHT;
                        pTS = TurretStates.IDOL;

                        isTurretStateFinished = true;

                        break;

                }

                switch (cLS)
                {

                    case BEGIN:

                        lift.setPower(0);

                        cLS = LiftStates.IDOL;
                        pLS = LiftStates.BEGIN;

                        break;

                    case IDOL:

                        lift.setPower(0);

                        cLS = LiftStates.LOWERING;
                        pLS = LiftStates.IDOL;

                        isLiftStateFinished = true;

                        break;

                }

                switch (cAS)
                {

                    case BEGIN:

                        arm.setPower(0.1);

                        cAS = ArmStates.IDOL;
                        pAS = ArmStates.BEGIN;

                        break;

                    case IDOL:

                        arm.setPower(0.1);

                        isArmStateFinished = true;

                        break;

                }

                switch (cCS)
                {

                    case BEGIN:

                        leftClaw.setPower(0);
                        rightClaw.setPower(0);

                        cCS = ClawStates.IDOL;
                        pCS = ClawStates.BEGIN;

                        break;

                    case IDOL:

                        leftClaw.setPower(0);
                        rightClaw.setPower(0);

                        isClawStateFinished = true;

                        break;

                }

                if (isDriveStateFinished && isFoundationStateFinished && isTurretStateFinished && isLiftStateFinished && isArmStateFinished && isClawStateFinished)
                {

                    cOS = AutoStates.NAVIGATE_TO_STONE;
                    pOS = AutoStates.BEGIN;

                    isDriveStateFinished = false;
                    isFoundationStateFinished= false;
                    isTurretStateFinished = false;
                    isLiftStateFinished = false;
                    isArmStateFinished = false;
                    isClawStateFinished = false;

                    isFirstDrive = true;
                    isFirstFoundation = true;
                    isFirstTurret = true;
                    isFirstLift = true;
                    isFirstArm = true;
                    isFirstClaw = true;

                }

            case NAVIGATE_TO_STONE:

                switch (cDS)
                {

                        case RIGHT:

                            if(alternate == 1)
                            {

                                if (isFirstDrive == true)
                                {

                                    dT.rightNoStop(0.25, 900);

                                    isFirstDrive = false;

                                }

                                if (!leftFront.isBusy() && !rightFront.isBusy() && !leftBack.isBusy() && !rightBack.isBusy())
                                {

                                    dT.kill();

                                    cDS = DriveStates.FORWARD;
                                    pDS = DriveStates.RIGHT;
                                    alternate = 2;

                                    dT.forwardNoStop(0.25, 500);

                                    break;

                                }

                            }

                            if(alternate == 2)
                            {

                                if ((!leftFront.isBusy() && !rightFront.isBusy() && !leftBack.isBusy() && !rightBack.isBusy()))
                                {

                                    dT.kill();

                                    cDS = DriveStates.IDOL;
                                    pDS = DriveStates.RIGHT;

                                    isDriveStateFinished = true;

                                    break;

                                }

                            }



                    case FORWARD:

                        if (!leftFront.isBusy() && !rightFront.isBusy() && !leftBack.isBusy() && !rightBack.isBusy())
                        {

                            dT.kill();

                            cDS = DriveStates.RIGHT;
                            pDS = DriveStates.FORWARD;

                            dT.rightNoStop(0.25, 325);

                            break;

                        }

                }

                switch (cFGS)
                {

                    case IDOL:

                        dT.release();

                        cFGS = FoundationGrabberStates.IDOL;
                        pFGS = FoundationGrabberStates.IDOL;

                        isFoundationStateFinished = true;

                        break;

                }

                switch (cTS)
                {

                    case TURN_RIGHT:

                        if(isFirstTurret == true)
                        {

                            a.tRightNoStop(0.25, 950);

                            isFirstTurret= false;

                        }

                        if(!turret.isBusy())
                        {

                            turret.setPower(0);

                            cTS = TurretStates.IDOL;
                            pTS = TurretStates.TURN_RIGHT;

                            isTurretStateFinished = true;

                            break;

                        }

                }

                switch (cLS)
                {

                    case LOWERING:

                        if(isFirstLift == true)
                        {

                            lift.setPower(0.5);

                            isFirstLift = false;

                            runtime.reset();

                        }

                        if(runtime.milliseconds() >= 750)
                        {

                            lift.setPower(0);

                            cLS = LiftStates.IDOL;
                            pLS = LiftStates.LOWERING;

                            break;

                        }


                    case IDOL:

                        lift.setPower(0);

                        cLS = LiftStates.LOWERING;
                        pLS = LiftStates.IDOL;

                        isLiftStateFinished = true;

                        break;


                }

                switch (cAS)
                {

                    case RAISE:

                        if(isFirstArm == true)
                        {

                            a.rUpNoStop(0.25, 500);

                        }

                        if(!arm.isBusy())
                        {

                            arm.setPower(0);

                            cAS = ArmStates.LOWER;
                            pAS = ArmStates.RAISE;

                            a.rDownNoStop(0.25, 300);

                            break;

                        }

                    case LOWER:

                        if(!arm.isBusy())
                        {

                            arm.setPower(0);

                            cAS = ArmStates.IDOL;
                            pAS = ArmStates.LOWER;

                            isArmStateFinished = true;

                            break;

                        }

                }

                switch (cCS)
                {

                    case IDOL:

                        leftClaw.setPower(0);
                        rightClaw.setPower(0);

                        isClawStateFinished = true;

                        break;

                }

                if (isDriveStateFinished && isFoundationStateFinished && isTurretStateFinished && isLiftStateFinished && isArmStateFinished && isClawStateFinished)
                {

                    cOS = AutoStates.NAVIGATE_TO_STONE;
                    pOS = AutoStates.BEGIN;

                    isDriveStateFinished = false;
                    isFoundationStateFinished= false;
                    isTurretStateFinished = false;
                    isLiftStateFinished = false;
                    isArmStateFinished = false;
                    isClawStateFinished = false;

                    isFirstDrive = true;
                    isFirstFoundation = true;
                    isFirstTurret = true;
                    isFirstLift = true;
                    isFirstArm = true;
                    isFirstClaw = true;

                }

            case GRAB_STONE:

                leftClaw.setPower(-1);
                rightClaw.setPower(1);

        }

    }

    void right (double spd, int tic)
    {

        if (!leftFront.isBusy() && !rightFront.isBusy() && !leftBack.isBusy() && !rightBack.isBusy())
        {

            dT.kill();

        }

    }

}