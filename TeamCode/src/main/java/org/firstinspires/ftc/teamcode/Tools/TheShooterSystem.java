package org.firstinspires.ftc.teamcode.Tools;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TheShooterSystem {
    private final TheArtifactBasketSystem basketSystem;
    private final DcMotorEx shooterMotorLeft;
    private final DcMotorEx shooterMotorRight;

    static private final int SHOOTING_TIME = 2000;
    static private final int MOTOR_SPEED = 1450;

    private ElapsedTime myTimer = new ElapsedTime();

    private ShooterState currentState = ShooterState.IDLE;

    public ShooterState getCurrentState() {
        return currentState;
    }


    public enum ShooterState {
        IDLE,
        RELEASING,
        SPEEDUP,
        SHOOTING
    }

    public TheShooterSystem(TheArtifactBasketSystem basket, DcMotorEx leftMotor, DcMotorEx rightMotor) {
        basketSystem = basket;
        shooterMotorLeft = leftMotor;
        shooterMotorRight = rightMotor;

    }

    public void StartShooterMotors()
    {
        shooterMotorLeft.setVelocity(MOTOR_SPEED);
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setVelocity(MOTOR_SPEED);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StopShooterMotors()
    {
        shooterMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void shootGreen() throws InterruptedException {
        StartShooterMotors();
        basketSystem.ReleaseGreen();
        currentState = ShooterState.RELEASING;
    }

    public void shootPurple() throws InterruptedException {
        if (basketSystem.hasPurple1)
        {
            shootPurple1();
        }
        else if (basketSystem.hasPurple2)
        {
            shootPurple2();
        }
    }

    public void shootPurple1() throws InterruptedException {
        StartShooterMotors();
        basketSystem.ReleasePurple1();
        currentState = ShooterState.RELEASING;
    }

    public void shootPurple2() throws InterruptedException {
        StartShooterMotors();
        basketSystem.ReleasePurple2();
        currentState = ShooterState.RELEASING;
    }

    public void Update() throws InterruptedException {
        switch (currentState) {
            case IDLE:
                // Do nothing
                break;
            case RELEASING:
                if (basketSystem.getCurrentState() == TheArtifactBasketSystem.BasketState.FREE)
                {
                    currentState = ShooterState.SPEEDUP;
                    myTimer.reset();
                }
                break;
            case SPEEDUP:
                if (myTimer.milliseconds() >= 500) {
                    currentState = ShooterState.SHOOTING;
                    myTimer.reset();
                }
                break;
            case SHOOTING:
                if (myTimer.milliseconds() >= SHOOTING_TIME) {
                    basketSystem.CloseShooter();
                    StopShooterMotors();
                    currentState = ShooterState.IDLE;
                }
                break;
        }
    }
}
