package org.firstinspires.ftc.teamcode.Tools;

import android.graphics.Color;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TheIntakeSystem {
    private final Motor intakeMotorLeft;
    private final Motor intakeMotorRight;
    private final Servo servoMotor;
    private final TheArtifactBasketSystem basketSystem;
    private final ColorRangeSensor colorSensor;

    private ElapsedTime myTimer = new ElapsedTime();

    static public final int INTAKE_TIME = 300;
    static public final int CLOSING_TIME = 500;

    public enum IntakeState {
        IDLE,
        INTAKING,
        LOADING_GREEN,
        LOADING_PURPLE1,
        LOADING_PURPLE2,
        UPLOADING,
        CLOSING,
        SPITTING
    }

    private IntakeState currentState = IntakeState.IDLE;

    public IntakeState getCurrentState() {
        return currentState;
    }


    public TheIntakeSystem(Motor leftMotor, Motor rightMotor, Servo servo, TheArtifactBasketSystem basket, ColorRangeSensor colour) {
        intakeMotorLeft = leftMotor;
        intakeMotorRight = rightMotor;
        servoMotor = servo;
        basketSystem = basket;
        colorSensor = colour;
        servoMotor.setPosition(0.0);
    }

    public void Update() throws InterruptedException {
        switch (currentState) {
            case IDLE:
                // Do nothing
                break;
            case INTAKING:
                if (colorSensor.getDistance(DistanceUnit.CM) <= 2.0)
                {
                    int red = colorSensor.red();
                    int green = colorSensor.green();
                    int blue = colorSensor.blue();
                    float[] hsv = new float[3];
                    Color.RGBToHSV(red, green, blue, hsv);
                    if (hsv[0] < 195.0)
                    {
                        if (!basketSystem.hasGreen)
                        {
                            basketSystem.ReceiveGreen();
                            currentState = IntakeState.LOADING_GREEN;
                        }
                        else {
                            // Already has green, stop intake
                            intakeMotorLeft.set(-1.0);
                            intakeMotorRight.set(-1.0);
                            currentState = IntakeState.SPITTING;
                        }
                    } else {
                        if (!basketSystem.hasPurple1)
                        {
                            basketSystem.ReceivePurple1();
                            currentState = IntakeState.LOADING_PURPLE1;
                        }
                        else {
                            if (!basketSystem.hasPurple2) {
                                basketSystem.ReceivePurple2();
                                currentState = IntakeState.LOADING_PURPLE2;
                            } else {
                                // Already has both purples, stop intake
                                intakeMotorLeft.set(-1.0);
                                intakeMotorRight.set(-1.0);
                                currentState = IntakeState.SPITTING;
                            }
                        }
                    }
                }
                break;
            case LOADING_GREEN:
                basketSystem.Update();
                if (basketSystem.getCurrentState() == TheArtifactBasketSystem.BasketState.RECEIVED_GREEN) {
                    servoMotor.setPosition(1.0);
                    myTimer.reset();
                    currentState = IntakeState.UPLOADING;
                }
                break;
            case LOADING_PURPLE1:
                basketSystem.Update();
                if (basketSystem.getCurrentState() == TheArtifactBasketSystem.BasketState.RECEIVED_PURPLE1) {
                    servoMotor.setPosition(1.0);
                    myTimer.reset();
                    currentState = IntakeState.UPLOADING;
                }
                break;
            case LOADING_PURPLE2:
                basketSystem.Update();
                if (basketSystem.getCurrentState() == TheArtifactBasketSystem.BasketState.RECEIVED_PURPLE2) {
                    servoMotor.setPosition(1.0);
                    myTimer.reset();
                    currentState = IntakeState.UPLOADING;
                }
                break;
            case UPLOADING:
                if (myTimer.milliseconds() >= INTAKE_TIME)
                {
                    basketSystem.CloseIntake();
                    myTimer.reset();
                    currentState = IntakeState.CLOSING;
                }
                break;
            case CLOSING:
                if (myTimer.milliseconds() >= CLOSING_TIME)
                {
                    servoMotor.setPosition(0.0);
                    intakeMotorLeft.set(0.0);
                    intakeMotorRight.set(0.0);
                    currentState = IntakeState.IDLE;
                }
                break;
            case SPITTING:
                if (colorSensor.getDistance(DistanceUnit.CM) >= 2.5)
                {
                    intakeMotorLeft.set(0.0);
                    intakeMotorRight.set(0.0);
                    currentState = IntakeState.IDLE;
                }
                break;
        }
    }

    public void stopIntake() throws InterruptedException {
        intakeMotorLeft.set(0.0);
        intakeMotorRight.set(0.0);
        servoMotor.setPosition(0.0);
        currentState = IntakeState.IDLE;
    }

    public boolean isIntaking() {
        return currentState != IntakeState.IDLE;
    }

    public void intake() throws InterruptedException {
        intakeMotorLeft.set(1.0);
        intakeMotorRight.set(1.0);
        currentState = IntakeState.INTAKING;
    }

    public void toggle_intake() throws InterruptedException
    {
        if (currentState == IntakeState.IDLE)
        {
            intake();
        }
        else
        {
            stopIntake();
        }
    }
}
