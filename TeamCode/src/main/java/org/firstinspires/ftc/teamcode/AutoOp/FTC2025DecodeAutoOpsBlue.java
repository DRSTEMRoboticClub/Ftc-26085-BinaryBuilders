package org.firstinspires.ftc.teamcode.AutoOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Tools.CameraController;
import org.firstinspires.ftc.teamcode.Tools.TheArtifactBasketSystem;
import org.firstinspires.ftc.teamcode.Tools.TheIntakeSystem;
import org.firstinspires.ftc.teamcode.Tools.TheShooterSystem;



@Autonomous
public class FTC2025DecodeAutoOpsBlue extends LinearOpMode {

    private TheArtifactBasketSystem basketSystem;
    private TheIntakeSystem intakeSystem;

    private TheShooterSystem shooterSystem;
    private DcMotorEx shooter_left;
    private DcMotorEx shooter_right;

    private MecanumDrive drive;
    private CameraController cameraController;
    private ColorRangeSensor colour_sensor;

    private MecanumDrivetrain drivetrain;



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
        drivetrain = new MecanumDrivetrain(hardwareMap, telemetry);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot initialisation
        initialisation();

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            drivetrain.turn_to(45, 0.5, 2);
        }
    }
}