package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.tools.ColourMatcher;
import org.firstinspires.ftc.teamcode.tools.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.tools.ControlPad;

@TeleOp
public class TheWheelsOnTheBusOp extends LinearOpMode {
    static final double TURN_RATE = Math.PI / 4; // 45 degrees each bumper hit
    static final String teamColour = "Red";

    // Servos
    private Servo intakeAngleServo;

    // Sensors
    private ColorRangeSensor colourSensor;

    // Drivetrain
    private MecanumDrivetrain driveTrain;

    // Control pad
    private ControlPad controlPad_1;

    // Robot Arm
    private TheArmOnTheBus missionArm;

    // Initialise robot
    public void autoBotRollout() throws InterruptedException
    {
        // Declare our motors
        // Create motors
        DcMotorEx frontLeftMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorC");
        DcMotorEx backLeftMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorD");
        DcMotorEx frontRightMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorB");
        DcMotorEx backRightMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorA");
        DcMotorEx armSlideMotor = (DcMotorEx)hardwareMap.dcMotor.get("MotorE");
        DcMotorEx armMotorLeft = (DcMotorEx)hardwareMap.dcMotor.get("MotorF");
        DcMotorEx armMotorRight = (DcMotorEx)hardwareMap.dcMotor.get("MotorG");
        DcMotor intakeFlapWheelMotor = hardwareMap.dcMotor.get("MotorH");
        DistanceSensor sensorLeft = hardwareMap.get(DistanceSensor.class, "DistanceL");
        DistanceSensor sensorRight = hardwareMap.get(DistanceSensor.class, "DistanceR");

        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Create servos
        intakeAngleServo = hardwareMap.servo.get("IntakeAngleServo");

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

        // drivetrain
        driveTrain = new MecanumDrivetrain(telemetry, imuSensor, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, sensorLeft, sensorRight);
        driveTrain.initialise();

        // Control pad
        controlPad_1 = new ControlPad(telemetry, gamepad1);

        // mission arm
        missionArm = new TheArmOnTheBus(telemetry, armMotorLeft, armMotorRight, armSlideMotor, intakeAngleServo, intakeFlapWheelMotor, colourSensor, teamColour);
        missionArm.initialise();
    }

    // Make the robot start grabbing samples from the pool
    public void sampleRobbery() throws InterruptedException {

        // Set the arm position to aim the pool

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


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing

            if (controlPad_1.is_left_bumper_pressed())
            {
                missionArm.lift_arm(135);
                Thread.sleep(1000);
                missionArm.extend_arm(1.0f);
            }
            else if (controlPad_1.is_right_bumper_pressed())
            {
                missionArm.extend_arm(0.0f);
                Thread.sleep(1000);
                missionArm.lift_arm(0);
            }

            ControlPad.JoyStickStatus right_joy_stick = controlPad_1.right_joystick_x();
            if (right_joy_stick == ControlPad.JoyStickStatus.LEFT)
            {
                driveTrain.turn_left(TURN_RATE);
            }
            if (right_joy_stick == ControlPad.JoyStickStatus.RIGHT)
            {
                driveTrain.turn_right(TURN_RATE);
            }

            driveTrain.run(x, y);
            missionArm.update();
            telemetry.update();
        }
    }
}

