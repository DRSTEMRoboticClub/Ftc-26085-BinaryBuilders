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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Tools.TheArtifactBasketSystem;
import org.firstinspires.ftc.teamcode.Tools.TheIntakeSystem;
import org.firstinspires.ftc.teamcode.Tools.TheShooterSystem;
import android.graphics.Color;

@TeleOp
public class FTC2025DecodeTeleOps extends LinearOpMode {

    private TheArtifactBasketSystem basketSystem;
    private TheIntakeSystem intakeSystem;

    private TheShooterSystem shooterSystem;
    private MotorEx shooter_left;
    private MotorEx shooter_right;

    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;
    private GamepadEx driveGamepad;
    private MecanumDrive drive;

    private ColorRangeSensor colour_sensor;

    private IMU imu;

    // Initialise robot hardware
    public void initialise() throws InterruptedException {
        ServoImplEx the_basket_servo = hardwareMap.get(ServoImplEx.class, "basket");
        Servo the_shutter1 = hardwareMap.get(Servo.class, "shutter1");
        Servo the_shutter2 = hardwareMap.get(Servo.class, "shutter2");
        Servo the_shutter3 = hardwareMap.get(Servo.class, "shutter3");
        Servo the_shutter4 = hardwareMap.get(Servo.class, "shutter4");
        shooter_left = new MotorEx(hardwareMap, "shooterleft");
        shooter_right = new MotorEx(hardwareMap, "shooterright");
        shooter_left.setRunMode(Motor.RunMode.VelocityControl);
        shooter_right.setRunMode(Motor.RunMode.VelocityControl);
        shooter_left.setInverted(true);
        shooter_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        Motor intake_motor_left = new Motor(hardwareMap, "intakeleft");
        intake_motor_left.setInverted(true);
        Motor intake_motor_right = new Motor(hardwareMap, "intakeright");
        Servo intake_servo = hardwareMap.get(Servo.class, "intake");
        colour_sensor = (ColorRangeSensor) hardwareMap.colorSensor.get("colourblind");
        basketSystem = new TheArtifactBasketSystem(the_basket_servo, the_shutter1, the_shutter2, the_shutter3, the_shutter4);
        intakeSystem = new TheIntakeSystem(intake_motor_left, intake_motor_right, intake_servo, basketSystem, colour_sensor);
        shooterSystem = new TheShooterSystem(basketSystem, shooter_left, shooter_right);
        frontLeft = new MotorEx(hardwareMap, "frontleft");
        frontRight = new MotorEx(hardwareMap, "frontright");
        backLeft = new MotorEx(hardwareMap, "backleft");
        backRight = new MotorEx(hardwareMap, "backright");
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        driveGamepad = new GamepadEx(gamepad1);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void startShooterMotors() {
        shooter_left.set(0.38);
        shooter_right.set(0.38);
    }

    public void stopShooterMotors() {
        shooter_left.set(0.0);
        shooter_right.set(0.0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot initialisation
        initialise();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double yaw = orientation.getYaw(AngleUnit.DEGREES);

            if (!gamepad1.right_bumper) {
                drive.driveFieldCentric(
                        -driveGamepad.getLeftX()*0.8,
                        -driveGamepad.getLeftY()*0.8,
                        -driveGamepad.getRightX()*0.8,
                        yaw);
            } else {
                drive.driveFieldCentric(
                        -driveGamepad.getLeftX()*0.15,
                        -driveGamepad.getLeftY()*0.15,
                        -driveGamepad.getRightX()*0.15,
                        yaw);
            }

            if (gamepad1.a) {
                startShooterMotors();
                shooterSystem.shootGreen();
            }

            if (gamepad1.b) {
                startShooterMotors();
                shooterSystem.shootPurple1();
            }

            if (gamepad1.x) {
                intakeSystem.intake();
            }

            if (gamepad1.y) {
                startShooterMotors();
                shooterSystem.shootPurple2();
            }

            if (gamepad1.left_bumper) {
                //rotate basket left
            }
            if (gamepad1.right_bumper) {
                //rotate basket right
            }

            if (gamepad2.a)
            {
                basketSystem.ReleasePurple1();
            }

            if (gamepad2.b)
            {
                basketSystem.ReleasePurple2();
            }

            if (gamepad2.x)
            {
                basketSystem.ReleaseGreen();
            }


            telemetry.addData("Distance: ", colour_sensor.getDistance(DistanceUnit.CM));
            int red = colour_sensor.red();
            int green = colour_sensor.green();
            int blue = colour_sensor.blue();

            float[] hsv = new float[3];
            Color.RGBToHSV(red, green, blue, hsv);

            // hsv[0] = hue, hsv[1] = saturation, hsv[2] = value
            telemetry.addData("Hue: ", hsv[0]);
            telemetry.addData("Saturation: ", hsv[1]);
            telemetry.addData("Value: ", hsv[2]);
            telemetry.addData("Yaw Angle", "%.2f", yaw);


            telemetry.update();
            basketSystem.Update();
            intakeSystem.Update();
        }
    }
}
 