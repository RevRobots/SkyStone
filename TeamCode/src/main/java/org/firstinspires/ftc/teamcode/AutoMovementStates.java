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

    /*Now we set up our enumerations for our state machine.
     *The enums include the entire auto program, drive train,
     *turret, lift, arm, and claw.
     */

    private enum AutoStates
    {

        BEGIN, NAVIGATE_TO_STONE, GRAB_STONE, MOVE_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, NAVIGATE_TO_SECOND_STONE, GRAB_SECOND_STONE, MOVE_TO_FOUNDATION_SSECOND, PLACE_SECOND_STONE, PARK

    }

    AutoStates cOS = AutoStates.BEGIN;
    AutoStates pOS;

    boolean isFirstRun = true;

    String currentState;

    private enum DriveStates
    {

        BEGIN, FORWARD, BACKWARD, RIGHT, LEFT, TURN_RIGHT, TURN_LEFT, IDOL

    }

    DriveStates cDS = DriveStates.BEGIN;
    DriveStates pDS;

    Boolean isFirstDrive = true;

    Boolean isDriveStateFinished = false;

    private enum FoundationGrabberStates
    {

        BEGIN, GRAB, RELEASE, IDOL

    }

    FoundationGrabberStates cFGS = FoundationGrabberStates.BEGIN;
    FoundationGrabberStates pFGS;

    Boolean isFirstFoundation = true;

    Boolean isFoundationStateFinished = false;

    private enum TurretStates
    {

        BEGIN, TURN_RIGHT, TURN_LEFT, IDOL

    }

    TurretStates cTS = TurretStates.BEGIN;
    TurretStates pTS;

    Boolean isFirstTurret = true;

    Boolean isTurretStateFinished = false;

    private enum LiftStates
    {

        BEGIN, RAISE, LOWER, IDOL

    }

    LiftStates cLS = LiftStates.BEGIN;
    LiftStates pLS;

    Boolean isFirstLift = true;

    Boolean isLiftStateFinished = false;

    private enum ArmStates
    {

        BEGIN, RAISE, LOWER, IDOL

    }

    ArmStates cAS = ArmStates.BEGIN;
    ArmStates pAS;

    Boolean isFirstArm = true;

    Boolean isArmStateFinished = false;

    private enum ClawStates
    {

        BEGIN, GRABBING, RELEASING, GRABBED, RELEASED, IDOL

    }

    ClawStates cCS = ClawStates.BEGIN;
    ClawStates pCS;

    Boolean isFirstClaw = true;

    Boolean isClawStateFinished = false;

    /*Constructor for our class to be used in our auto programs.
     *Assigns the variables that we set up above using the
     *configuration to assign motors to motors and so on.
     */

    public AutoMovementStates (DcMotor lF, DcMotor rF, DcMotor lB, DcMotor rB, Servo lFo, Servo rFo, DcMotor t, DcMotor l, DcMotor ar, CRServo lC, CRServo rC, DriveTrain dt, Armstrong armstrong, ElapsedTime rt)
    {

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

    void run1 ()
    {

        switch (cOS)
        {

            case BEGIN:

                begin();

                break;

            case NAVIGATE_TO_STONE:

                navigateToStone();

                break;

        }

    }

    String getState ()
    {

        return currentState;

    }

    void run2(int extraTick) throws InterruptedException
    {

        switch (cOS)
        {

            case MOVE_TO_FOUNDATION:

                switch (cDS)
                {

                    case RIGHT:

                        if (isFirstDrive == true)
                        {

                            dT.right(0.25, 3120 + extraTick);

                            isFirstDrive = false;

                        }

                        if (!leftFront.isBusy() && !rightFront.isBusy() && !leftBack.isBusy() && !rightBack.isBusy())
                        {

                            dT.kill();

                            cDS = DriveStates.IDOL;
                            pDS = DriveStates.RIGHT;

                            break;

                        }

                    case IDOL:

                        dT.kill();

                        cDS = DriveStates.TURN_RIGHT;
                        pDS = DriveStates.IDOL;

                        isDriveStateFinished = true;

                        break;

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

                        if (isFirstTurret == true)
                        {

                            a.tRightNoStop(0.25, 950);

                            isFirstTurret = false;

                        }

                        if (!turret.isBusy())
                        {

                            turret.setPower(0);

                            cTS = TurretStates.IDOL;
                            pTS = TurretStates.TURN_RIGHT;

                            runtime.reset();

                            break;

                        }

                    case IDOL:

                        turret.setPower(0);

                        if (runtime.milliseconds() >= 250)
                        {

                            cTS = TurretStates.TURN_LEFT;
                            pTS = TurretStates.IDOL;

                            a.tLeftNoStop(0.25, 950);

                            break;

                        }

                    case TURN_LEFT:

                        if (!turret.isBusy())
                        {

                            turret.setPower(0);

                            cTS = TurretStates.IDOL;
                            pTS = TurretStates.TURN_LEFT;

                            isTurretStateFinished = true;

                            break;

                        }

                }

                switch(cLS)
                {

                    case IDOL:

                        lift.setPower(0);

                        cLS = LiftStates.IDOL;
                        pLS = LiftStates.IDOL;

                        isLiftStateFinished = true;

                        break;

                }

                switch (cAS)
                {

                    case IDOL:

                        arm.setPower(0.1);

                        cAS = ArmStates.IDOL;
                        pAS = ArmStates.IDOL;

                        isArmStateFinished = true;

                        break;

                }

                switch (cCS)
                {

                    case IDOL:

                        leftClaw.setPower(-1);
                        rightClaw.setPower(1);

                        cCS = ClawStates.IDOL;
                        pCS = ClawStates.IDOL;

                        break;

                }

                if (isDriveStateFinished && isFoundationStateFinished && isTurretStateFinished && isLiftStateFinished && isArmStateFinished && isClawStateFinished)
                {

                    cOS = AutoStates.MOVE_TO_FOUNDATION;
                    pOS = AutoStates.NAVIGATE_TO_STONE;

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

                    break;

                }

            case PLACE_STONE:

                a.unclamp(1, 250);

                cOS = AutoStates.MOVE_FOUNDATION;
                pOS = AutoStates.PLACE_STONE;

                break;

        }

    }

    void begin ()
    {

        currentState = "begin";

        cOS = AutoStates.NAVIGATE_TO_STONE;

    }

    void navigateToStone ()
    {

        if (cDS != DriveStates.IDOL && cLS != LiftStates.IDOL && cAS != ArmStates.IDOL)
        {

            switch (cDS)
            {

                case BEGIN:

                    cDSBegin();

                    break;

                case FORWARD:

                    cDSForward();

                    break;

                case IDOL:

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

                case IDOL:

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

                case IDOL:

                    break;

            }

        } else
        {

            cOS = AutoStates.MOVE_TO_FOUNDATION;

        }

    }

    void cDSBegin ()
    {

        dT.forwardNoStop(.25, 850);

        cDS = DriveStates.FORWARD;

    }

    void cDSForward ()
    {

        if (!leftFront.isBusy() && !rightFront.isBusy() && !leftBack.isBusy() && !rightBack.isBusy())
        {

            dT.kill();

            cDS = DriveStates.IDOL;

        }

    }

    void cLSBegin ()
    {

        runtime.reset();

        lift.setPower(0.75);

        cLS = LiftStates.LOWER;

    }

    void cLSLower ()
    {

        if (runtime.milliseconds() >= 500)
        {

            lift.setPower(0);

            cLS = LiftStates.IDOL;

        }

    }

    void cASBegin ()
    {

        a.rUpNoStop(0.25, 250);

        cAS = ArmStates.RAISE;

    }

    void cASRaise ()
    {

        if (!arm.isBusy())
        {

            arm.setPower(0);

            cAS = ArmStates.IDOL;

        }

    }

}