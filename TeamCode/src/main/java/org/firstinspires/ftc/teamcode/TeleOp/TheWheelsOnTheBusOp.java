package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.tools.ColourMatcher;

@TeleOp
public class TheWheelsOnTheBusOp extends LinearOpMode {
    static int ARM_POSITION_SAMPLE = 300;
    static int ARM_POSITION_POWER = 5;
    static int WHEELS_FULL_VELOCITY = 1500;
    static int SLIDE_FULL_SIZE = 1250;
    static int SLIDE_POWER = 50;

    // Devices
    // Motors
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx armBottomMotor;
    private DcMotorEx sliderMotorLeft;
    private DcMotorEx sliderMotorRight;

    // Servos
    private Servo intakeAngleServo;
    private Servo intakeFlapWheelServo;

    // Sensors
    private ColorRangeSensor colourSensor;

    // Colour Matcher
    private ColourMatcher colourMatcher;

    // Initialise robot
    public void autoBotRollout() throws InterruptedException
    {
        // Declare our motors
        // Create motors
        frontLeftMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorC");
        backLeftMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorD");
        frontRightMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorB");
        backRightMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorA");
        armBottomMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorE");
        sliderMotorLeft = (DcMotorEx)hardwareMap.dcMotor.get("SliderMotorLeft");
        sliderMotorRight = (DcMotorEx)hardwareMap.dcMotor.get("SliderMotorRight");

        // Create servos
        intakeAngleServo = hardwareMap.servo.get("IntakeAngleServo");
        intakeFlapWheelServo = hardwareMap.servo.get("IntakeSampleServo");

        // Create Sensors
        colourSensor = (ColorRangeSensor) hardwareMap.colorSensor.get("ColourSensor");

        // Initialisation
        // Initialise Servos
        intakeAngleServo.setPosition(1.0);
        intakeFlapWheelServo.setPosition(0.5);

        // Initialise Motors
        // Front left motor
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // back right motor
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // front left motor
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // back left motor
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // arm lifting motor
        armBottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armBottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // slider motor left
        sliderMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // slider motor right
        sliderMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Define the colours
        colourMatcher.AddColour("Blue", 0.0, 0.0, 1.0, 1.0);
        colourMatcher.AddColour("Red", 1.0, 0.0, 0.0, 1.0);
        colourMatcher.AddColour("Yellow", 1.0, 1.0, 0.0, 1.0);
    }

    // Make the robot start grabbing samples from the pool
    public void sampleRobbery() throws InterruptedException {

        // Set the arm position to aim the pool
        armBottomMotor.setTargetPosition(ARM_POSITION_SAMPLE);
        armBottomMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBottomMotor.setPower(ARM_POSITION_POWER);
        while (armBottomMotor.isBusy())
        {
            telemetry.addData("Position", armBottomMotor.getCurrentPosition());
            telemetry.update();
        }

        // Turn the intake to aim to the pool
        intakeAngleServo.setPosition(0.0);

        // Extend the linear slider
        sliderMotorLeft.setTargetPosition(SLIDE_FULL_SIZE);
        sliderMotorRight.setTargetPosition(SLIDE_FULL_SIZE);
        sliderMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorLeft.setPower(SLIDE_POWER);
        sliderMotorRight.setPower(SLIDE_POWER);
        while (sliderMotorLeft.isBusy() || sliderMotorRight.isBusy())
        {
            telemetry.addData("Left", sliderMotorLeft.getCurrentPosition());
            telemetry.addData("Right", sliderMotorRight.getCurrentPosition());
            telemetry.update();
        }

        // Turn the intake till a sample is collected
        intakeFlapWheelServo.setPosition(1.0);
        while (colourSensor.getDistance(DistanceUnit.MM) > 50.0)
        {
            Thread.sleep(100);
        }
        intakeFlapWheelServo.setPosition(0.5);

        // Check colour
        NormalizedRGBA c = colourSensor.getNormalizedColors();
        if (colourMatcher.ClosestColour(c.red, c.green, c.blue, c.alpha).get_name() == "Blue")
        {
            
        } else if (colourMatcher.ClosestColour(c.red, c.green, c.blue, c.alpha).get_name() == "Red") {
            
        }

        // Pull back the linear slider

    }

    // Deliver the sample into the basket
    public void amazonDelivery() throws InterruptedException {

        // lift the arm up

        // extend the slider

        // turn the intake

        // turn the intake wheels in the opposite direction

        // turn the intake back

        // pull back the slider

        // lower the arm to aim the pool
    }

    // Climb in the endgame
    public void humptyDumptyRobot() {
        // To be figured out
    }


    @Override
    public void runOpMode() throws InterruptedException {

        // Robot initialisation
        autoBotRollout();

        waitForStart();

        if (isStopRequested()) return;

        double maxVelocityFrontRight = 0.0;
        double maxVelocityFrontLeft = 0.0;
        double maxVelocityBackRight = 0.0;
        double maxVelocityBackLeft = 0.0;

        //sliderMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sliderMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // scaling
            double scaledFrontLeftPower = frontLeftPower * frontLeftPower * frontLeftPower;
            double scaledBackLeftPower = backLeftPower * backLeftPower * backLeftPower;
            double scaledFrontRightPower = frontRightPower * frontRightPower * frontRightPower;
            double scaledBackRightPower = backRightPower * backRightPower * backRightPower;

            frontLeftMotor.setVelocity(scaledFrontLeftPower*WHEELS_FULL_VELOCITY);
            backLeftMotor.setVelocity(scaledBackLeftPower*WHEELS_FULL_VELOCITY);
            frontRightMotor.setVelocity(scaledFrontRightPower*WHEELS_FULL_VELOCITY);
            backRightMotor.setVelocity(scaledBackRightPower*WHEELS_FULL_VELOCITY);

            if (gamepad1.left_bumper)
            {
                //sampleRobbery();
            }

            if (frontRightMotor.getVelocity() > maxVelocityFrontRight)
            {
                maxVelocityFrontRight = frontRightMotor.getVelocity();
            }

            if (backRightMotor.getVelocity() > maxVelocityBackRight)
            {
                maxVelocityBackRight = backRightMotor.getVelocity();
            }

            if (frontLeftMotor.getVelocity() > maxVelocityFrontLeft)
            {
                maxVelocityFrontLeft = frontLeftMotor.getVelocity();
            }

            if (backLeftMotor.getVelocity() > maxVelocityBackLeft)
            {
                maxVelocityBackLeft = backLeftMotor.getVelocity();
            }

            telemetry.addData("FrontRight", maxVelocityFrontRight);
            telemetry.addData("BackRight", maxVelocityBackRight);
            telemetry.addData("FrontLeft", maxVelocityFrontLeft);
            telemetry.addData("BackLeft", maxVelocityBackLeft);

            /*
            if (gamepad1.left_trigger > 0)
            {
                sliderMotorLeft.setVelocity(50);
                sliderMotorRight.setVelocity(50);
            }
            else if (gamepad1.right_trigger > 0)
            {
                sliderMotorLeft.setVelocity(-50);
                sliderMotorRight.setVelocity(-50);
            }
            else
            {
                sliderMotorLeft.setVelocity(0);
                sliderMotorRight.setVelocity(0);
            }
            telemetry.addData("Left", sliderMotorLeft.getCurrentPosition());
            telemetry.addData("Right", sliderMotorRight.getCurrentPosition());
            */
            telemetry.update();
        }
    }
}

