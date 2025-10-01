package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tools.TheArtifactBasketSystem;

@TeleOp
public class FTC2025DecodeTeleOps extends LinearOpMode {

    private TheArtifactBasketSystem basketSystem;

    // Initialise robot hardware
    public void initialise() throws InterruptedException {
        Servo the_basket_servo = hardwareMap.get(Servo.class, "basket");
        Servo the_shutter1 = hardwareMap.get(Servo.class, "shutter1");
        Servo the_shutter2 = hardwareMap.get(Servo.class, "shutter2");
        Servo the_shutter3 = hardwareMap.get(Servo.class, "shutter3");
        Servo the_shutter4 = hardwareMap.get(Servo.class, "shutter4");
        basketSystem = new TheArtifactBasketSystem(the_basket_servo, the_shutter1, the_shutter2, the_shutter3, the_shutter4);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot initialisation
        initialise();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                basketSystem.ReleasePurple1();
            }

            if (gamepad1.b) {
                basketSystem.ReleasePurple2();
            }

            if (gamepad1.x) {
                basketSystem.ReleaseGreen();
            }
            telemetry.update();
        }
    }
}
