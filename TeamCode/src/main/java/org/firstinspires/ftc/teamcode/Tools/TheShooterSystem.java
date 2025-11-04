package org.firstinspires.ftc.teamcode.Tools;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TheShooterSystem {
    private final TheArtifactBasketSystem basketSystem;
    private final DcMotorEx shooterMotorLeft;
    private final DcMotorEx shooterMotorRight;

    static public final int SHOOTING_TIME = 2000;

    private ElapsedTime myTimer = new ElapsedTime();

    private ShooterState currentState = ShooterState.IDLE;

    private enum ShooterState {
        IDLE,
        RELEASING,
        SHOOTING
    }

    public TheShooterSystem(TheArtifactBasketSystem basket, DcMotorEx leftMotor, DcMotorEx rightMotor) {
        basketSystem = basket;
        shooterMotorLeft = leftMotor;
        shooterMotorRight = rightMotor;

    }

    public void StartShooterMotors()
    {
        shooterMotorLeft.setVelocity(280);
        shooterMotorRight.setVelocity(280);
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        myTimer.reset();
        currentState = ShooterState.RELEASING;
    }

    public void shootPurple1() throws InterruptedException {
        StartShooterMotors();
        basketSystem.ReleasePurple1();
        myTimer.reset();
        currentState = ShooterState.RELEASING;
    }

    public void shootPurple2() throws InterruptedException {
        StartShooterMotors();
        basketSystem.ReleasePurple2();
        myTimer.reset();
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
                    currentState = ShooterState.SHOOTING;
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
