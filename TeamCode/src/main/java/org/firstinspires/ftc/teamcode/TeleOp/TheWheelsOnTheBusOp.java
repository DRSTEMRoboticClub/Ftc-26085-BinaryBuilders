package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.teamcode.tools.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.tools.ControlPad;
import org.firstinspires.ftc.teamcode.tools.TheArmOnTheBus;
import org.firstinspires.ftc.teamcode.tools.TheIntakOnTheArm;

@TeleOp
public class TheWheelsOnTheBusOp extends LinearOpMode {
    static final double TURN_RATE = Math.PI / 4; // 45 degrees each bumper hit
    static final int DISTANCE_TO_CORNER = 160;
    static final int TOLERANCE_TO_CORNER = 20;
    static final float SPEED_CORNER = 0.25f;
    static final String teamColour = "Red";

    enum State
    {
        READY,
        ROBBERY,
        ROBBED,
        DELIVERING,
        CLIMBING
    }

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

    private State state;

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

        state = State.READY;
    }

    // Make the robot start grabbing samples from the pool
    public void sampleRobbery() throws InterruptedException {
        missionArm.lift_arm(21);
        missionArm.extend_arm(0.2f);
        missionArm.intake_down();
        state = State.ROBBERY;
    }

    // Make the robot start grabbing samples from the pool
    public void backToReadyPosition() throws InterruptedException {
        missionArm.intake_up();
        missionArm.extend_arm(0.0f);
        missionArm.lift_arm(0);
    }

    public void robberySuccess() throws InterruptedException {
        backToReadyPosition();
        state = State.ROBBED;
    }

    // Make the robot start grabbing samples from the pool
    public void cancelRobbery() throws InterruptedException {
        backToReadyPosition();
        state = State.READY;
    }

    // Deliver the sample into the basket
    public void amazonDelivery() throws InterruptedException {
        driveTrain.corner(DISTANCE_TO_CORNER, TOLERANCE_TO_CORNER, SPEED_CORNER);
        missionArm.lift_arm(135);
        missionArm.extend_arm(1.0f);
        missionArm.intake_delivery();
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

            ControlPad.JoyStickStatus right_joy_stick = controlPad_1.right_joystick_x();
            if (right_joy_stick == ControlPad.JoyStickStatus.LEFT)
            {
                driveTrain.turn_left(TURN_RATE);
            }
            if (right_joy_stick == ControlPad.JoyStickStatus.RIGHT)
            {
                driveTrain.turn_right(TURN_RATE);
            }


            switch (state)
            {
                case READY:
                    if (controlPad_1.is_left_bumper_pressed())
                    {
                        sampleRobbery();
                    }
                    break;
                case ROBBERY:
                    if (controlPad_1.is_left_bumper_pressed())
                    {
                        cancelRobbery();
                    }
                    else
                    {
                        if (missionArm.getState() == TheIntakOnTheArm.State.GRABBED)
                        {
                            robberySuccess();
                            state = State.ROBBED;
                        }
                    }
                    break;
                case ROBBED:
                    if (controlPad_1.is_right_bumper_pressed())
                    {
                        amazonDelivery();
                        state = State.DELIVERING;
                    }
                    break;
                case DELIVERING:
                    if (missionArm.getState() == TheIntakOnTheArm.State.IDLE)
                    {
                        backToReadyPosition();
                        state = State.READY;
                    }
            }

            driveTrain.run(x, y, 1.0);
            missionArm.update();
            telemetry.update();
        }
    }
}

