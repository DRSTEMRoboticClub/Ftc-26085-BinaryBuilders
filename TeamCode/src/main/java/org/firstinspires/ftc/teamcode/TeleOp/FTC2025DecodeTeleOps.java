package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tools.CameraController;
import org.firstinspires.ftc.teamcode.Tools.TheArtifactBasketSystem;
import org.firstinspires.ftc.teamcode.Tools.TheIntakeSystem;
import org.firstinspires.ftc.teamcode.Tools.TheShooterSystem;
import android.graphics.Color;

@TeleOp
public class FTC2025DecodeTeleOps extends LinearOpMode {

    private TheArtifactBasketSystem basketSystem;
    private TheIntakeSystem intakeSystem;

    private TheShooterSystem shooterSystem;
    private DcMotorEx shooter_left;
    private DcMotorEx shooter_right;

    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;
    private GamepadEx driveGamepad1;
    private GamepadEx driveGamepad2;
    private MecanumDrive drive;

    private CameraController cameraController;

    private ColorRangeSensor colour_sensor;

    private ElapsedTime runtime = new ElapsedTime();

    private IMU imu;

    // Initialise robot hardware
    public void initialise() throws InterruptedException {
        ServoImplEx the_basket_servo = hardwareMap.get(ServoImplEx.class, "basket");
        Servo the_shutter1 = hardwareMap.get(Servo.class, "shutter1");
        Servo the_shutter2 = hardwareMap.get(Servo.class, "shutter2");
        Servo the_shutter3 = hardwareMap.get(Servo.class, "shutter3");
        Servo the_shutter4 = hardwareMap.get(Servo.class, "shutter4");
        Servo the_camera_servo = hardwareMap.get(Servo.class, "cameraServo");
        shooter_left = (DcMotorEx)hardwareMap.get(DcMotor.class, "shooterleft");
        shooter_right = (DcMotorEx)hardwareMap.get(DcMotor.class, "shooterright");
        shooter_left.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor intake_motor_left = new Motor(hardwareMap, "intakeleft");
        Motor intake_motor_right = new Motor(hardwareMap, "intakeright");
        intake_motor_right.setInverted(true);
        Servo intake_servo = hardwareMap.get(Servo.class, "intake");
        colour_sensor = (ColorRangeSensor) hardwareMap.colorSensor.get("colourblind");
        basketSystem = new TheArtifactBasketSystem(the_basket_servo, the_shutter1, the_shutter2, the_shutter3, the_shutter4, shooter_left, shooter_right);
        intakeSystem = new TheIntakeSystem(intake_motor_left, intake_motor_right, intake_servo, basketSystem, colour_sensor);
        shooterSystem = new TheShooterSystem(basketSystem, shooter_left, shooter_right);
        cameraController = new CameraController(the_camera_servo, hardwareMap, telemetry);
        frontLeft = new MotorEx(hardwareMap, "frontleft");
        frontRight = new MotorEx(hardwareMap, "frontright");
        backLeft = new MotorEx(hardwareMap, "backleft");
        backRight = new MotorEx(hardwareMap, "backright");
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        driveGamepad1 = new GamepadEx(gamepad1);
        driveGamepad2 = new GamepadEx(gamepad2);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot initialisation
        initialise();

        waitForStart();

        if (isStopRequested()) return;

        runtime.reset();

        while (opModeIsActive()) {
            if (runtime.milliseconds() > 119000) {
                drive.stop();
                break;
            }

            double speed = gamepad1.right_bumper || gamepad2.right_bumper ? 0.15 : 0.8;
            if (driveGamepad1.getLeftX() != 0.0
                    || driveGamepad1.getLeftY() != 0.0
                    || driveGamepad1.getRightX() != 0.0) {
                drive.driveRobotCentric(-driveGamepad1.getLeftX() * speed,
                        -driveGamepad1.getLeftY() * speed,
                        -driveGamepad1.getRightX() * speed);
            }
            else {
                drive.driveRobotCentric(driveGamepad2.getLeftX() * speed,
                        driveGamepad2.getLeftY() * speed, -driveGamepad2.getRightX() * speed);
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper)
            {
                basketSystem.OpenIntake();
                basketSystem.OpenShooter();
                intakeSystem.spit();
            }

            if (gamepad1.left_trigger < 0.25) {
                if (gamepad1.a) {
                    shooterSystem.shootGreen();
                }

                if (gamepad1.b) {
                    shooterSystem.shootPurple1();
                }

                if (gamepad1.y) {
                    shooterSystem.shootPurple2();
                }
            }
            else
            {
                if (gamepad1.a) {
                    shooterSystem.shootGreenLong();
                }

                if (gamepad1.b) {
                    shooterSystem.shootPurple1Long();
                }

                if (gamepad1.y) {
                    shooterSystem.shootPurple2Long();
                }
            }


            if (gamepad2.x) {
                intakeSystem.toggle_intake();
                Thread.sleep(100);
            }


            if (gamepad1.dpad_down) {
                cameraController.Down();
            }

            if (gamepad1.dpad_right) {
                cameraController.Front();
            }

            telemetry.update();
            basketSystem.Update();
            intakeSystem.Update();
            shooterSystem.Update();
        }
    }
}
 