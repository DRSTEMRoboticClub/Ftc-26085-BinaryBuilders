package org.firstinspires.ftc.teamcode.AutoOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Tools.CameraController;
import org.firstinspires.ftc.teamcode.Tools.TheArtifactBasketSystem;
import org.firstinspires.ftc.teamcode.Tools.TheIntakeSystem;
import org.firstinspires.ftc.teamcode.Tools.TheShooterSystem;



@Autonomous
public class FTC2025DecodeAutoOpsRed extends LinearOpMode {

    private TheArtifactBasketSystem basketSystem;
    private TheIntakeSystem intakeSystem;

    private TheShooterSystem shooterSystem;
    private DcMotorEx shooter_left;
    private DcMotorEx shooter_right;

    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;
    private MecanumDrive drive;
    private CameraController cameraController;
    private ColorRangeSensor colour_sensor;
    private IMU imu;

    private MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;

    private void initialiseOdometry()
    {
        Translation2d frontLeftLocation = new Translation2d(-0.168, 0.168);
        Translation2d frontRightLocation = new Translation2d(0.168, 0.168);
        Translation2d backLeftLocation = new Translation2d(-0.168, -0.168);
        Translation2d backRightLocation = new Translation2d(0.168, -0.168);
        kinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        odometry = new MecanumDriveOdometry(kinematics, new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)), new Pose2d(5.0, 13.5, new Rotation2d()));
    }

    private void initialisation() throws InterruptedException
    {
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
        intake_motor_left.setInverted(true);
        Motor intake_motor_right = new Motor(hardwareMap, "intakeright");
        Servo intake_servo = hardwareMap.get(Servo.class, "intake");
        colour_sensor = (ColorRangeSensor) hardwareMap.colorSensor.get("colourblind");
        basketSystem = new TheArtifactBasketSystem(the_basket_servo, the_shutter1, the_shutter2, the_shutter3, the_shutter4);
        intakeSystem = new TheIntakeSystem(intake_motor_left, intake_motor_right, intake_servo, basketSystem, colour_sensor);
        shooterSystem = new TheShooterSystem(basketSystem, shooter_left, shooter_right);
        cameraController = new CameraController(the_camera_servo, hardwareMap, telemetry);
        frontLeft = new MotorEx(hardwareMap, "frontleft");
        frontRight = new MotorEx(hardwareMap, "frontright");
        backLeft = new MotorEx(hardwareMap, "backleft");
        backRight = new MotorEx(hardwareMap, "backright");
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot initialisation
        initialisation();

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

        }
    }
}