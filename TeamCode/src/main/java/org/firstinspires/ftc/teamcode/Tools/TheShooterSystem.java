package org.firstinspires.ftc.teamcode.Tools;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TheShooterSystem {
    private final TheArtifactBasketSystem basketSystem;
    private final Motor shooterMotorLeft;
    private final Motor shooterMotorRight;

    static public final int SHOOTING_TIME = 1000;

    private ElapsedTime myTimer = new ElapsedTime();

    private ShooterState currentState = ShooterState.IDLE;

    private enum ShooterState {
        IDLE,
        SHOOTING_GREEN,
        SHOOTING_PURPLE1,
        SHOOTING_PURPLE2
    }

    public TheShooterSystem(TheArtifactBasketSystem basket, MotorEx leftMotor, MotorEx rightMotor) {
        basketSystem = basket;
        shooterMotorLeft = leftMotor;
        shooterMotorRight = rightMotor;
    }

    public void shootGreen() throws InterruptedException {
        basketSystem.ReleaseGreen();
        myTimer.reset();
        currentState = ShooterState.SHOOTING_GREEN;
    }

    public void shootPurple1() throws InterruptedException {
        basketSystem.ReleasePurple1();
        myTimer.reset();
        currentState = ShooterState.SHOOTING_PURPLE1;
    }

    public void shootPurple2() throws InterruptedException {
        basketSystem.ReleasePurple2();
        myTimer.reset();
        currentState = ShooterState.SHOOTING_PURPLE2;
    }

    public void Update() throws InterruptedException {
        switch (currentState) {
            case IDLE:
                // Do nothing
                break;
            case SHOOTING_GREEN:
            case SHOOTING_PURPLE1:
            case SHOOTING_PURPLE2:
                if (myTimer.milliseconds() >= SHOOTING_TIME) {
                    basketSystem.CloseShooter();
                    currentState = ShooterState.IDLE;
                }
                break;
        }
    }
}
