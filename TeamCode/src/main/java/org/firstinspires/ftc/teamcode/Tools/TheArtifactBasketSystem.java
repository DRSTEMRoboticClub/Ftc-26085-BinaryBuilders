package org.firstinspires.ftc.teamcode.Tools;
import com.qualcomm.robotcore.hardware.Servo;
public class TheArtifactBasketSystem {
    private final Servo basketServo;
    private final Servo shutter1;
    private final Servo shutter2;
    private final Servo shutter3;
    private final Servo shutter4;

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
        Thread.sleep(500);
    }

    public void ReleasePurple2() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        basketServo.setPosition(0.0);
        OpenShooter();
    }

    public void ReleasePurple1() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        basketServo.setPosition(120.0/300.0);
        OpenShooter();
    }

    public void ReleaseGreen() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        basketServo.setPosition(250.0/300.0);
        OpenShooter();
    }

    public void ReceiveGreen() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        basketServo.setPosition(58.0/300.0);
        OpenIntake();
    }

    public void ReceivePurple2() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        basketServo.setPosition(185.0/300.0);
        OpenIntake();
    }

    public void ReceivePurple1() throws InterruptedException {
        CloseIntake();
        CloseShooter();
        basketServo.setPosition(1.0);
        OpenIntake();
    }
}
