package org.firstinspires.ftc.teamcode.Tools;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TheArtifactBasketSystem {
    private final ServoImplEx basketServo;
    private final Servo shutter1;
    private final Servo shutter2;
    private final Servo shutter3;
    private final Servo shutter4;
    private double servoPosition = 0.0;

    public boolean hasGreen = false;
    public boolean hasPurple1 = false;
    public boolean hasPurple2 = false;
    public boolean isIntakeShutterOpen = true;
    public boolean isShooterShutterOpen = true;

    static public final int WAIT_SHUTTER_MILLISECONDS = 750;
    static public final int WAIT_BASKET_MILLISECONDS = 750;

    static private final double GREEN_RECEIVE_POSITION = 60.0 / 300.0;
    static private final double PURPLE2_RECEIVE_POSITION = 180.0 / 300.0;
    static private final double PURPLE1_RECEIVE_POSITION = 1.0;
    static private final double PURPLE2_RELEASE_POSITION = 0.0;
    static private final double PURPLE1_RELEASE_POSITION = 120.0 / 300.0;
    static private final double GREEN_RELEASE_POSITION = 237.0 / 300.0;

    private ElapsedTime myTimer = new ElapsedTime();

    public enum BasketState {
        RECEIVING_GREEN,
        RECEIVING_PURPLE2,
        RECEIVING_PURPLE1,
        RELEASING_PURPLE2,
        RELEASING_PURPLE1,
        RELEASING_GREEN,
        PRE_RECEIVING_GREEN,
        PRE_RECEIVING_PURPLE2,
        PRE_RECEIVING_PURPLE1,
        PRE_RELEASING_PURPLE2,
        PRE_RELEASING_PURPLE1,
        PRE_RELEASING_GREEN,
        POST_RECEIVING_GREEN,
        POST_RECEIVING_PURPLE2,
        POST_RECEIVING_PURPLE1,
        POST_RELEASING_PURPLE2,
        POST_RELEASING_PURPLE1,
        POST_RELEASING_GREEN,
        RECEIVED_GREEN,
        RECEIVED_PURPLE2,
        RECEIVED_PURPLE1,
        FREE
    }

    private BasketState currentState = BasketState.FREE;

    private void turnServoTo(double position) {
        if (position < 0.0) {
            position = 0.0;
        } else if (position > 1.0) {
            position = 1.0;
        }
        basketServo.setPosition(position);
        servoPosition = position;
    }

    public TheArtifactBasketSystem(ServoImplEx the_basket_servo, Servo the_shutter1, Servo the_shutter2, Servo the_shutter3, Servo the_shutter4) throws InterruptedException {
        basketServo = the_basket_servo;
        shutter1 = the_shutter1;
        shutter2 = the_shutter2;
        shutter3 = the_shutter3;
        shutter4 = the_shutter4;
        CloseShooter();
        CloseIntake();
        basketServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        shutter1.setPosition(0.0);
        shutter2.setPosition(0.0);
        shutter3.setPosition(0.0);
        shutter4.setPosition(0.0);
        turnServoTo(0.0);
    }

    public void OpenIntake() throws InterruptedException {
        shutter3.setPosition(1.0);
        shutter4.setPosition(1.0);
        isIntakeShutterOpen = true;
    }

    public void CloseIntake() throws InterruptedException {
        if (isIntakeShutterOpen) {
            if (servoPosition + 0.1 <= 1.0) {
                turnServoTo(servoPosition + 0.1);
            } else {
                turnServoTo(servoPosition - 0.1);
            }
        Thread.sleep(100);
        shutter3.setPosition(0.0);
        shutter4.setPosition(0.0);
        isIntakeShutterOpen = false;
        }
    }

    public void OpenShooter() {
        shutter1.setPosition(1.0);
        shutter2.setPosition(1.0);
        isShooterShutterOpen = true;
    }

    public void CloseShooter() throws InterruptedException {
        if (isShooterShutterOpen) {
            Thread.sleep(100);
            shutter1.setPosition(0.0);
            shutter2.setPosition(0.0);
            isShooterShutterOpen = false;
        }
    }

    public void ReleasePurple2() throws InterruptedException {
        if (isIntakeShutterOpen || isShooterShutterOpen)
        {
            CloseShooter();
            CloseIntake();
            currentState = BasketState.PRE_RELEASING_PURPLE2;
        }
        else
        {
            turnServoTo(PURPLE2_RELEASE_POSITION);
            currentState = BasketState.RELEASING_PURPLE2;
        }

        myTimer.reset();

    }

    public void ReleasePurple1() throws InterruptedException {
        if (isIntakeShutterOpen || isShooterShutterOpen)
        {
            CloseShooter();
            CloseIntake();
            currentState = BasketState.PRE_RELEASING_PURPLE1;
        }
        else
        {
            turnServoTo(PURPLE1_RELEASE_POSITION);
            currentState = BasketState.RELEASING_PURPLE1;
        }
        myTimer.reset();
    }

    public void ReleaseGreen() throws InterruptedException {
        if (isIntakeShutterOpen || isShooterShutterOpen)
        {
            CloseShooter();
            CloseIntake();
            currentState = BasketState.PRE_RELEASING_GREEN;
        }
        else
        {
            turnServoTo(GREEN_RELEASE_POSITION);
            currentState = BasketState.RELEASING_GREEN;
        }
        myTimer.reset();
    }

    public void ReceiveGreen() throws InterruptedException {
        if (isIntakeShutterOpen || isShooterShutterOpen)
        {
            CloseIntake();
            CloseShooter();
            currentState = BasketState.PRE_RECEIVING_GREEN;
        }
        else
        {
            turnServoTo(GREEN_RECEIVE_POSITION);
            currentState = BasketState.RECEIVING_GREEN;
        }
        myTimer.reset();
    }

    public void ReceivePurple2() throws InterruptedException {
        if (isIntakeShutterOpen || isShooterShutterOpen)
        {
            CloseIntake();
            CloseShooter();
            currentState = BasketState.PRE_RECEIVING_PURPLE2;
        }
        else
        {
            turnServoTo(PURPLE2_RECEIVE_POSITION);
            currentState = BasketState.RECEIVING_PURPLE2;
        }
        myTimer.reset();
    }

    public void ReceivePurple1() throws InterruptedException {
        if (isIntakeShutterOpen || isShooterShutterOpen)
        {
            CloseIntake();
            CloseShooter();
            currentState = BasketState.PRE_RECEIVING_PURPLE1;
        }
        else
        {
            turnServoTo(PURPLE1_RECEIVE_POSITION);
            currentState = BasketState.RECEIVING_PURPLE1;
        }
        myTimer.reset();
    }

    public void Update() throws InterruptedException {
            switch (currentState) {
                case PRE_RECEIVING_GREEN:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        turnServoTo(GREEN_RECEIVE_POSITION);
                        myTimer.reset();
                        currentState = BasketState.RECEIVING_GREEN;
                    }
                    break;
                case RECEIVING_GREEN:
                    if (myTimer.milliseconds() > WAIT_BASKET_MILLISECONDS) {
                        myTimer.reset();
                        OpenIntake();
                        currentState = BasketState.POST_RECEIVING_GREEN;
                    }
                    break;
                case POST_RECEIVING_GREEN:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        currentState = BasketState.RECEIVED_GREEN;
                        hasGreen = true;
                    }
                    break;
                case PRE_RECEIVING_PURPLE2:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        turnServoTo(PURPLE2_RECEIVE_POSITION);
                        myTimer.reset();
                        currentState = BasketState.RECEIVING_PURPLE2;
                    }
                    break;
                case RECEIVING_PURPLE2:
                    if (myTimer.milliseconds() > WAIT_BASKET_MILLISECONDS) {
                        myTimer.reset();
                        OpenIntake();
                        currentState = BasketState.POST_RECEIVING_PURPLE2;
                    }
                    break;
                case POST_RECEIVING_PURPLE2:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        currentState = BasketState.RECEIVED_PURPLE2;
                        hasPurple2 = true;
                    }
                    break;
                case PRE_RECEIVING_PURPLE1:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        turnServoTo(PURPLE1_RECEIVE_POSITION);
                        myTimer.reset();
                        currentState = BasketState.RECEIVING_PURPLE1;
                    }
                    break;
                case RECEIVING_PURPLE1:
                    if (myTimer.milliseconds() > WAIT_BASKET_MILLISECONDS) {
                        myTimer.reset();
                        OpenIntake();
                        currentState = BasketState.POST_RECEIVING_PURPLE1;
                    }
                    break;
                case POST_RECEIVING_PURPLE1:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        currentState = BasketState.RECEIVED_PURPLE1;
                        hasPurple1 = true;
                    }
                    break;
                case PRE_RELEASING_GREEN:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        turnServoTo(GREEN_RELEASE_POSITION);
                        myTimer.reset();
                        currentState = BasketState.RELEASING_GREEN;
                    }
                    break;
                case RELEASING_GREEN:
                    if (myTimer.milliseconds() > WAIT_BASKET_MILLISECONDS) {
                        myTimer.reset();
                        OpenShooter();
                        currentState = BasketState.POST_RELEASING_GREEN;
                    }
                    break;
                case POST_RELEASING_GREEN:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        currentState = BasketState.FREE;
                        hasGreen = false;
                    }
                    break;
                case PRE_RELEASING_PURPLE2:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        myTimer.reset();
                        turnServoTo(PURPLE2_RELEASE_POSITION);
                        currentState = BasketState.RELEASING_PURPLE2;
                    }
                    break;
                case RELEASING_PURPLE2:
                    if (myTimer.milliseconds() > WAIT_BASKET_MILLISECONDS) {
                        myTimer.reset();
                        OpenShooter();
                        currentState = BasketState.POST_RELEASING_PURPLE2;
                    }
                    break;
                case POST_RELEASING_PURPLE2:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        currentState = BasketState.FREE;
                        hasPurple2 = false;
                    }
                    break;
                case PRE_RELEASING_PURPLE1:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        myTimer.reset();
                        turnServoTo(PURPLE1_RELEASE_POSITION);
                        currentState = BasketState.RELEASING_PURPLE1;
                    }
                    break;
                case RELEASING_PURPLE1:
                    if (myTimer.milliseconds() > WAIT_BASKET_MILLISECONDS) {
                        myTimer.reset();
                        OpenShooter();
                        currentState = BasketState.POST_RELEASING_PURPLE1;
                    }
                    break;
                case POST_RELEASING_PURPLE1:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        currentState = BasketState.FREE;
                        hasPurple1 = false;
                    }
                    break;
                case FREE:
                    break;

        }
    }

    public BasketState getCurrentState() { return currentState;}
}
