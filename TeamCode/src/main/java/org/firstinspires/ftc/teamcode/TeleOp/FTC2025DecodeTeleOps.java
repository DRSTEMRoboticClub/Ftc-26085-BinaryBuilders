package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tools.TheArtifactBasketSystem;
import org.firstinspires.ftc.teamcode.Tools.TheIntakeSystem;
import android.graphics.Color;

@TeleOp
public class FTC2025DecodeTeleOps extends LinearOpMode {

    private TheArtifactBasketSystem basketSystem;
    private TheIntakeSystem intakeSystem;
    private Motor shooter_left;
    private Motor shooter_right;

    private ColorRangeSensor colour_sensor;

    // Initialise robot hardware
    public void initialise() throws InterruptedException {
        ServoImplEx the_basket_servo = hardwareMap.get(ServoImplEx.class, "basket");
        Servo the_shutter1 = hardwareMap.get(Servo.class, "shutter1");
        Servo the_shutter2 = hardwareMap.get(Servo.class, "shutter2");
        Servo the_shutter3 = hardwareMap.get(Servo.class, "shutter3");
        Servo the_shutter4 = hardwareMap.get(Servo.class, "shutter4");
        shooter_left = new Motor(hardwareMap, "shooterleft");
        shooter_right = new Motor(hardwareMap, "shooterright");
        shooter_left.setRunMode(Motor.RunMode.RawPower);
        shooter_right.setRunMode(Motor.RunMode.RawPower);
        shooter_left.setInverted(true);
        Motor intake_motor_left = new Motor(hardwareMap, "intakeleft");
        intake_motor_left.setInverted(true);
        Motor intake_motor_right = new Motor(hardwareMap, "intakeright");
        Servo intake_servo = hardwareMap.get(Servo.class, "intake");
        colour_sensor = (ColorRangeSensor) hardwareMap.colorSensor.get("colourblind");
        basketSystem = new TheArtifactBasketSystem(the_basket_servo, the_shutter1, the_shutter2, the_shutter3, the_shutter4);
        intakeSystem = new TheIntakeSystem(intake_motor_left, intake_motor_right, intake_servo, basketSystem, colour_sensor);
    }

    public void startShooterMotors() {
        shooter_left.set(1.0);
        shooter_right.set(1.0);
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
            if (gamepad1.a) {
                startShooterMotors();
                Thread.sleep(1000);
                basketSystem.OpenShooter();

            }

            if (gamepad1.b) {
                basketSystem.CloseShooter();
                stopShooterMotors();
            }

            if (gamepad1.x) {
                intakeSystem.intake();
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


            telemetry.update();
            basketSystem.Update();
        }
    }
}
