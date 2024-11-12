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
    static final int TOLERANCE_TO_CORNER = 20;
    static final float SPEED_CORNER = 0.25f;
    static final String teamColour = "Red";

    enum State
    {
        READY,
        ROBBERY,
        ROBBED,
        DELIVERING,
        DROPPING,
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
    private ControlPad controlPad_2;

    // Robot Arm
    private TheArmOnTheBus missionArm;

    private State state;

    private float moveSpeed = 1.0f;

    private boolean autoPilotOn = false;

    //private int distanceToCorner = 400;
    private int distanceToCorner = 750;

    // Initialise robot
    public void autoBotRollout() throws InterruptedException
    {
        // Declare our motors
        // Create motors
        autoPilotOn = false;
        moveSpeed = 1.f;
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
        missionArm.lift_arm(20);
        missionArm.extend_arm(0.2f);
        missionArm.intake_down();
        state = State.ROBBERY;
    }

    // Make the arm and intake extends
    public void pushArm()
    {
        if (missionArm.getArmExtend() < 0.45f)
        {
            missionArm.lift_arm(missionArm.getArmAngle() + 0.5f);
            missionArm.extend_arm(missionArm.getArmExtend() + 0.02f);
        }
    }

    public void pullArm()
    {
        missionArm.lift_arm(missionArm.getArmAngle() - 0.5f);
        missionArm.extend_arm(missionArm.getArmExtend() - 0.02f);
    }

    // Make the robot start grabbing samples from the pool
    public void backToReadyPosition() throws InterruptedException {
        autoPilotOn = false;
        missionArm.intake_up();
        missionArm.extend_arm(0.01f);
        Thread.sleep(500);
        missionArm.lift_arm(1);
        moveSpeed = 1.f;
    }

    public void downToReadyPosition() throws InterruptedException {
        autoPilotOn = false;
        missionArm.intake_down();
        missionArm.extend_arm(0.01f);
        Thread.sleep(1000);
        missionArm.intake_up();
        missionArm.lift_arm(1);
        moveSpeed = 1.f;
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
        missionArm.lift_arm(135);
        Thread.sleep(1500);
        missionArm.extend_arm(0.99f);
        missionArm.set_servo_position(0.25f);
    }

    public void autoAmazonDelivery() throws InterruptedException {
        int limit = 500;
        while (limit > 0)
        {
             boolean alignment_success = driveTrain.corner(distanceToCorner, TOLERANCE_TO_CORNER, SPEED_CORNER);
             if (alignment_success)
             {
                 amazonDelivery();
                 Thread.sleep(2000);
                 missionArm.intake_delivery();
                 state = State.DROPPING;
                 return;
             }
             limit--;
        }
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

        while (opModeIsActive()) {
            ControlPad.JoyStickStatus right_joy_stick = controlPad_1.right_joystick_x();
            telemetry.addData("Arm extension", missionArm.getArmExtend());
            telemetry.addData("State",state.name());
            if (right_joy_stick == ControlPad.JoyStickStatus.LEFT)
            {
                driveTrain.turn_left(TURN_RATE);
            }
            if (right_joy_stick == ControlPad.JoyStickStatus.RIGHT)
            {
                driveTrain.turn_right(TURN_RATE);
            }

            if (gamepad1.x)
            {
                missionArm.intake_spit();
                cancelRobbery();
            }

            if (gamepad2.y)
            {
                autoPilotOn = true;
            }
            else if (gamepad2.x)
            {
                autoPilotOn = false;
            }

            switch (state)
            {
                case READY:
                    moveSpeed = 0.95f;
                    if (controlPad_1.is_left_bumper_pressed())
                    {
                        sampleRobbery();
                    }
                    break;
                case ROBBERY:
                    // Slow down the robot
                    moveSpeed = 0.35f;

                    // made the arm adjustable
                    if (gamepad1.left_trigger > 0)
                    {
                        pushArm();
                    }
                    else if (gamepad1.right_trigger > 0)
                    {
                        pullArm();
                    }

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
                    moveSpeed = 0.95f;;
                    // In case the sample drops out
                    if (controlPad_1.is_left_bumper_pressed())
                    {
                        sampleRobbery();
                    }

                    // Up arm to get ready for delivering the sample
                    if (gamepad2.left_bumper)
                    {
                        amazonDelivery();
                        state = State.DELIVERING;
                    }

                    // Up arm to get ready for delivering the sample
                    if (gamepad2.right_bumper )
                    {
                        autoAmazonDelivery();
                    }
                    break;

                case DELIVERING:
                    moveSpeed = 0.2f;

                    if (gamepad2.a)
                    {
                        missionArm.intake_delivery();
                        state = State.DROPPING;
                    }
                    break;
                case DROPPING:
                    moveSpeed = 0.f;
                    if (missionArm.getState() == TheIntakOnTheArm.State.IDLE)
                    {
                        downToReadyPosition();
                        autoPilotOn = false;
                        state = State.READY;
                    }
            }

            // Choose auto mode or manual mode
            if (autoPilotOn)
            {
                if (gamepad2.dpad_right)
                {
                    distanceToCorner += 50;
                }
                else if (gamepad2.dpad_left)
                {
                    distanceToCorner -= 50;
                }
                driveTrain.corner(distanceToCorner, TOLERANCE_TO_CORNER, SPEED_CORNER);
            }
            else
            {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                driveTrain.run(x, y, moveSpeed);
            }

            missionArm.update();
            telemetry.update();
        }
    }
}

