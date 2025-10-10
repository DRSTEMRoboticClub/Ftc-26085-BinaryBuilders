package org.firstinspires.ftc.teamcode.Tools;

import android.graphics.Color;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TheIntakeSystem {
    private final Motor intakeMotorLeft;
    private final Motor intakeMotorRight;
    private final Servo servoMotor;
    private final TheArtifactBasketSystem basketSystem;
    private final ColorRangeSensor colorSensor;

    public TheIntakeSystem(Motor leftMotor, Motor rightMotor, Servo servo, TheArtifactBasketSystem basket, ColorRangeSensor colour) {
        intakeMotorLeft = leftMotor;
        intakeMotorRight = rightMotor;
        servoMotor = servo;
        basketSystem = basket;
        colorSensor = colour;
        servoMotor.setPosition(0.0);
    }

    public void intake() throws InterruptedException {
        intakeMotorLeft.set(1.0);
        intakeMotorRight.set(1.0);

        while (colorSensor.getDistance(DistanceUnit.CM) > 2.0) {
            Thread.sleep(50);
        }

        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);

        if (hsv[0] < 195.0) {
            basketSystem.ReceiveGreen();
            while (basketSystem.getCurrentState() != TheArtifactBasketSystem.BasketState.RECEIVED_GREEN) {
                Thread.sleep(50);
                basketSystem.Update();
            }
        } else {
            if (basketSystem.hasPurple2)
            {
                basketSystem.ReceivePurple1();
                while (basketSystem.getCurrentState() != TheArtifactBasketSystem.BasketState.RECEIVED_PURPLE1) {
                    Thread.sleep(50);
                    basketSystem.Update();
                }
            }
            else
            {
                basketSystem.ReceivePurple2();
                while (basketSystem.getCurrentState() != TheArtifactBasketSystem.BasketState.RECEIVED_PURPLE2) {
                    Thread.sleep(50);
                    basketSystem.Update();
                }
            }

        }
        servoMotor.setPosition(1.0);
        Thread.sleep(300);
        basketSystem.CloseIntake();
        Thread.sleep(500);
        servoMotor.setPosition(0.0);
        intakeMotorLeft.set(0.0);
        intakeMotorRight.set(0.0);
    }

}
