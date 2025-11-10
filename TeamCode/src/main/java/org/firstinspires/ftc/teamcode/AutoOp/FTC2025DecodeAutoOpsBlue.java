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

    private Integer sequenceId = 0;
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

    private void shootSequence() throws InterruptedException
    {
        switch (sequenceId)
        {
            case 21:
                shooterSystem.shootGreen();
                while (shooterSystem.getCurrentState() != TheShooterSystem.ShooterState.IDLE) {shooterSystem.Update();}
                shooterSystem.shootPurple1();
                while (shooterSystem.getCurrentState() != TheShooterSystem.ShooterState.IDLE) {shooterSystem.Update();}
                shooterSystem.shootPurple2();
                while (shooterSystem.getCurrentState() != TheShooterSystem.ShooterState.IDLE) {shooterSystem.Update();}
                break;
            case 22:
                shooterSystem.shootPurple1();
                while (shooterSystem.getCurrentState() != TheShooterSystem.ShooterState.IDLE) {shooterSystem.Update();}
                shooterSystem.shootGreen();
                while (shooterSystem.getCurrentState() != TheShooterSystem.ShooterState.IDLE) {shooterSystem.Update();}
                shooterSystem.shootPurple2();
                while (shooterSystem.getCurrentState() != TheShooterSystem.ShooterState.IDLE) {shooterSystem.Update();}
                break;
            case 23:
                shooterSystem.shootPurple1();
                while (shooterSystem.getCurrentState() != TheShooterSystem.ShooterState.IDLE) {shooterSystem.Update();}
                shooterSystem.shootPurple2();
                while (shooterSystem.getCurrentState() != TheShooterSystem.ShooterState.IDLE) {shooterSystem.Update();}
                shooterSystem.shootGreen();
                while (shooterSystem.getCurrentState() != TheShooterSystem.ShooterState.IDLE) {shooterSystem.Update();}
                break;
            default:
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot initialisation
        initialisation();

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            sequenceId = cameraController.get_mission_tag();
            telemetry.addData("Mission Tag: ", sequenceId);
            telemetry.update();
            drivetrain.drive_forward(0.8, 2200);
            if (sequenceId == 0)
            {
                sequenceId = cameraController.get_mission_tag();
            }
            telemetry.addData("Mission Tag: ", sequenceId);
            telemetry.update();
            drivetrain.turn_to(-45, 0.3, 2);
            drivetrain.drive_backward(0.5, 300);
            drivetrain.turn_to(-45, 0.2, 2);
            shootSequence();
            drivetrain.turn_to(90, 0.5, 2);
            drivetrain.turn_to(90, 0.2, 2);
            drivetrain.drive_backward(0.5, 400);
            drivetrain.intake(150, 0.8);
            drivetrain.intake(150, 0.8);
            drivetrain.intake(150, 0.8);
            drivetrain.drive_backward(0.8, 950);
            drivetrain.turn_to(-45, 0.5, 2);
            drivetrain.turn_to(-45, 0.2, 2);
            shootSequence();
            drivetrain.turn_to(90, 0.5, 2);
            drivetrain.turn_to(90, 0.2, 2);
            drivetrain.right(0.6, 600);
            drivetrain.intake(150, 0.8);
            drivetrain.intake(150, 0.8);
            drivetrain.intake(150, 0.8);
            drivetrain.drive_backward(0.8, 950);
            drivetrain.left(0.6, 600);
            drivetrain.turn_to(-45, 0.5, 2);
            drivetrain.turn_to(-45, 0.2, 2);
            shootSequence();
            drivetrain.turn_to(90, 0.5, 2);
            drivetrain.turn_to(90, 0.2, 2);
            drivetrain.right(0.6, 1200);
            drivetrain.intake(150, 0.8);
            drivetrain.intake(150, 0.8);
            drivetrain.intake(150, 0.8);
            drivetrain.drive_backward(0.8, 950);
            drivetrain.left(0.6, 1200);
            drivetrain.turn_to(-45, 0.5, 2);
            drivetrain.turn_to(-45, 0.2, 2);
            shootSequence();
            drivetrain.turn_to(90, 0.5, 2);
            drivetrain.turn_to(90, 0.2, 2);
            drivetrain.right(0.6, 600);
        }
    }
}