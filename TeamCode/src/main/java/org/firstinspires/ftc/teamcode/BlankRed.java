package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous (name="Enter Name Here", group="Red")
@Disabled
public class BlankRed extends LinearOpMode
{

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AQUWr4X/////AAABme38EPssRkvls9+q/BGPYgxKXBXELWHMdkTcCqUqHeyDpyXGWFLCTABgDXEMGe1EmsnDQxmJ7WQ069J3YSv+kOcfq3g2EnwZr2O3DujsIU1nT0aXgLlAtQU2r7wWAgHvR9ADO5pe/q7MzCyhjSTQLCgizGFLgmqfre0A9rjYcXYbYw11R3P7VRHnL3QHn3QH2oFVQfMb+dIzmZkfv0cd5qWvdhjovYF8hpZ/HT7veIa8ZQ9CIQ0541pxplXVud80z1xWpjFGJPaoQGO+xKWZ8E+Zlu7z5umiaV1+ChGeJ9pPyIJn0LsnoIHumZoYb4di4tFygMPVmH8ChsTlGJjaPBSCRBFjxzBqsXmBZY7eCa6S";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

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

    Servo capstone;

    //Gyroscope
    BNO055IMU imu;

    //Robot Classes
    DriveTrain dT;
    Armstrong a;

    //State Machine Class
    RedStates rS;

    //Timer
    ElapsedTime runtime;

    //Stone counter
    int count = 1;
    //Extra distance for each stone
    int extraTick;

    //Boolean for if the skystone has been found
    boolean skystone = false;

    //The label of the object the program sees
    String label;

    @Override
    public void runOpMode()
    {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

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
        a = new Armstrong(gamepad1, gamepad2,  turret, lift, arm, leftClaw, rightClaw, capstone, imu);
        runtime = new ElapsedTime();

        //Set the motors to stop and stay still instead of removing all power and coasting
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Stops Robot
        dT.kill();
        dT.release();
        a.kill();

        //Setting up Red States
        rS = new RedStates(leftFront, rightFront, leftBack, rightBack, leftFoundation, rightFoundation, turret, lift, arm, leftClaw, rightClaw, dT, a, runtime);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        //Resets Timer
        runtime.reset();

        //If the play button is pressed
        //Uncomment to activate skystone detection
        /*if (opModeIsActive())
        {

            //While the state machine is still running
            while (rS.getState() != "finished")
            {

                //Run the state machine method
                rS.runFull();

                //Telemetry that displays the current state
                telemetry.addData("State>", rS.getState());

                //Display Telemetry
                telemetry.update();

            }

            //Loops the program
            while (opModeIsActive())
            {

                if (tfod != null)
                {

                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.

                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null)
                    {

                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.

                        int i = 0;

                        for (Recognition recognition : updatedRecognitions)
                        {

                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            //Sets the label to the name of the recognition
                            label = recognition.getLabel();

                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());

                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            telemetry.update();
                        }

                        //If label equals Skystone or if two stones have been detected
                        if(label == "Skystone" || count == 3)
                        {

                            //Telemetry for detecting the Skystone
                            telemetry.addData("Guess What", "Skystone Baby");

                            //Displays telemetry
                            telemetry.update();

                            //Sets the Skystone flag to true
                            skystone = true;

                            //Stops robot
                            dT.kill();

                            //Exits while loop
                            break;

                            //If the label detects Stone and the timer has been going for one second
                        } else if (label == "Stone" && runtime.milliseconds() >= 1000)
                        {

                            //Telemetry for not seeing a Skystone
                            telemetry.addData("Guess What", "Nothing!");

                            //Displays Telemetry
                            telemetry.update();

                            //Adds one to the count number
                            count++;

                            //Adds to the extra tick distance
                            extraTick = extraTick + 380;

                            //Moves left to next stone
                            dT.left(0.25, 380);

                            //Resets timer
                            runtime.reset();

                            //If the robot times out
                        } else if (runtime.milliseconds() >= 1500)
                        {

                            //Rotate arm up to create movement to help detection
                            a.rUp(0.25, 50);

                            //Resets timer
                            runtime.reset();

                        }

                        //Displays Telemetry
                        telemetry.update();

                    }
                }
            }
        }*/

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}