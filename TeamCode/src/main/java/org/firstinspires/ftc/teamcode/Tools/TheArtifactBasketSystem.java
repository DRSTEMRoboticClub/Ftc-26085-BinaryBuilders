package org.firstinspires.ftc.teamcode.Tools;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TheArtifactBasketSystem {
    private final Servo basketServo;
    private final Servo shutter1;
    private final Servo shutter2;
    private final Servo shutter3;
    private final Servo shutter4;

    static public final int WAIT_SHUTTER_MILLISECONDS = 500;
    static public final int WAIT_BASKET_MILLISECONDS = 500;

    ElapsedTime myTimer = new ElapsedTime();

    private enum BasketState {
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
        RELEASED_PURPLE2,
        RELEASED_PURPLE1,
        RELEASED_GREEN,
        FREE
    }

    private BasketState currentState = BasketState.FREE;

    public TheArtifactBasketSystem(Servo the_basket_servo, Servo the_shutter1, Servo the_shutter2, Servo the_shutter3, Servo the_shutter4) throws InterruptedException {
        basketServo = the_basket_servo;
        shutter1 = the_shutter1;
        shutter2 = the_shutter2;
        shutter3 = the_shutter3;
        shutter4 = the_shutter4;
        CloseShooter();
        CloseIntake();
        basketServo.setPosition(0.0);
    }

    public void OpenIntake() {
        shutter3.setPosition(1.0);
        shutter4.setPosition(1.0);
    }

    public void CloseIntake() {
        shutter3.setPosition(0.0);
        shutter4.setPosition(0.0);
    }

    public void OpenShooter() {
        shutter1.setPosition(1.0);
        shutter2.setPosition(1.0);
    }

    public void CloseShooter() throws InterruptedException {
        shutter1.setPosition(0.0);
        shutter2.setPosition(0.0);
    }

    public void ReleasePurple2() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        myTimer.reset();
        currentState = BasketState.PRE_RELEASING_PURPLE2;
    }

    public void ReleasePurple1() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        myTimer.reset();
        currentState = BasketState.PRE_RELEASING_PURPLE1;
    }

    public void ReleaseGreen() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        myTimer.reset();
        currentState = BasketState.PRE_RELEASING_GREEN;
    }

    public void ReceiveGreen() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        myTimer.reset();
        currentState = BasketState.PRE_RECEIVING_GREEN;
    }

    public void ReceivePurple2() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        myTimer.reset();
        currentState = BasketState.PRE_RECEIVING_PURPLE2;
    }

    public void ReceivePurple1() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        myTimer.reset();
        currentState = BasketState.PRE_RECEIVING_PURPLE1;
    }

    public void Update() throws InterruptedException {
            switch (currentState) {
                case PRE_RECEIVING_GREEN:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        basketServo.setPosition(58.0 / 300.0);
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
                    }
                    break;
                case PRE_RECEIVING_PURPLE2:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        basketServo.setPosition(185.0 / 300.0);
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
                    }
                    break;
                case PRE_RECEIVING_PURPLE1:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        basketServo.setPosition(1.0);
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
                    }
                    break;
                case PRE_RELEASING_GREEN:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        basketServo.setPosition(250.0 / 300.0);
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
                        currentState = BasketState.RELEASED_GREEN;
                    }
                    break;
                case PRE_RELEASING_PURPLE2:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        myTimer.reset();
                        basketServo.setPosition(0.0);
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
                        currentState = BasketState.RELEASED_PURPLE2;
                    }
                    break;
                case PRE_RELEASING_PURPLE1:
                    if (myTimer.milliseconds() > WAIT_SHUTTER_MILLISECONDS) {
                        myTimer.reset();
                        basketServo.setPosition(120.0 / 300.0);
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
                        currentState = BasketState.RELEASED_PURPLE1;
                    }
                    break;
                case FREE:
                    break;

        }
    }
}
