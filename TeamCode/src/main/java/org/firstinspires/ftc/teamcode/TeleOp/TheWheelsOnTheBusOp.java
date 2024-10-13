package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.tools.ColourMatcher;
import org.firstinspires.ftc.teamcode.tools.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.tools.ControlPad;

@TeleOp
public class TheWheelsOnTheBusOp extends LinearOpMode {
    static final int ARM_POSITION_SAMPLE = 300;
    static final int ARM_POSITION_POWER = 5;
    static final int SLIDE_FULL_SIZE = 1250;
    static final int SLIDE_POWER = 50;
    static final double TURN_RATE = Math.PI / 4; // 45 degrees each bumper hit

    // Servos
    private Servo intakeAngleServo;
    private Servo intakeFlapWheelServo;

    // Sensors
    private ColorRangeSensor colourSensor;

    // Colour Matcher
    private ColourMatcher colourMatcher;

    // Drivetrain
    private MecanumDrivetrain driveTrain;

    // Control pad
    private ControlPad controlPad_1;

    // Initialise robot
    public void autoBotRollout() throws InterruptedException
    {
        // Declare our motors
        // Create motors
        DcMotorEx frontLeftMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorC");
        DcMotorEx backLeftMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorD");
        DcMotorEx frontRightMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorB");
        DcMotorEx backRightMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorA");
        DcMotorEx armBottomMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorE");
        DcMotorEx sliderMotorLeft = (DcMotorEx)hardwareMap.dcMotor.get("SliderMotorLeft");
        DcMotorEx sliderMotorRight = (DcMotorEx)hardwareMap.dcMotor.get("SliderMotorRight");

        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Create servos
        intakeAngleServo = hardwareMap.servo.get("IntakeAngleServo");
        intakeFlapWheelServo = hardwareMap.servo.get("IntakeSampleServo");

        // Create Sensors
        colourSensor = (ColorRangeSensor) hardwareMap.colorSensor.get("ColourSensor");
        IMU imuSensor = hardwareMap.get(IMU.class, "imu");
        imuSensor.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

        // Initialisation
        // Initialise Servos
        intakeAngleServo.setPosition(1.0);
        intakeFlapWheelServo.setPosition(0.5);

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

        // drivetrain
        driveTrain = new MecanumDrivetrain(telemetry, imuSensor, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        driveTrain.initialise();

        // Control pad
        controlPad_1 = new ControlPad(telemetry, gamepad1);
    }

    // Make the robot start grabbing samples from the pool
    public void sampleRobbery() throws InterruptedException {

        // Set the arm position to aim the pool

        // Turn the intake to aim to the pool
        intakeAngleServo.setPosition(0.0);

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
        telemetry.update();
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

            if (controlPad_1.is_left_bumper_pressed())
            {
                driveTrain.turn_left(TURN_RATE);
            }
            else if (controlPad_1.is_right_bumper_pressed())
            {
                driveTrain.turn_right(TURN_RATE);
            }

            driveTrain.run(x, y);

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

