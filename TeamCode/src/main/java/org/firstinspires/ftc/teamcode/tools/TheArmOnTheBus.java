package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class TheArmOnTheBus {
    private final DcMotorEx leftArmMotor;
    private final DcMotorEx rightArmMotor;
    private final DcMotorEx slideMotor;
    private final Servo intakeServo;
    private float servoPosition;

    private final Telemetry telemetry;
    private TheIntakOnTheArm intake;

    private final int MAX_SLIDE = 3650;
    private final int MIN_SLIDE = 5;
    private final int ARM_MOTOR_MIN_ENCODE = 5;
    private final int ARM_MOTOR_ENCODE_COUNT = (int)Math.round(288.0 / 15 * 72);

    public TheArmOnTheBus(Telemetry the_telemetry, DcMotorEx left_arm_motor, DcMotorEx right_arm_motor, DcMotorEx slide_motor, Servo intake_servo, DcMotor intake_motor, ColorRangeSensor color_sensor, String teamColour)
    {
        telemetry = the_telemetry;
        leftArmMotor = left_arm_motor;
        rightArmMotor = right_arm_motor;
        slideMotor = slide_motor;
        intakeServo = intake_servo;
        servoPosition = 1.f;
        intake = new TheIntakOnTheArm(the_telemetry, intake_motor, color_sensor, teamColour);
    }

    public void initialise()
    {
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArmMotor.setTargetPositionTolerance(5);
        leftArmMotor.setCurrentAlert(4.0, CurrentUnit.AMPS);
        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setTargetPositionTolerance(5);
        rightArmMotor.setCurrentAlert(4.0, CurrentUnit.AMPS);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setTargetPositionTolerance(20);
        slideMotor.setCurrentAlert(8.0, CurrentUnit.AMPS);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        servoPosition = 1.f;
        intake.initialise();
    }

    public void lift_arm(float angle)
    {
        int targetArmPosition = Math.max(ARM_MOTOR_MIN_ENCODE, Math.round(angle / 360 * ARM_MOTOR_ENCODE_COUNT));
        leftArmMotor.setTargetPosition(targetArmPosition);
        rightArmMotor.setTargetPosition(targetArmPosition);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmMotor.setPower(1.0);
        rightArmMotor.setPower(1.0);
    }

    public void extend_arm(float percentage)
    {
        int slide_target = Math.max(Math.round(percentage * MAX_SLIDE), MIN_SLIDE);
        slideMotor.setTargetPosition(slide_target);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1.0);
    }

    public TheIntakOnTheArm.State getState()
    {
        return intake.getState();
    }

    public void intake_down()
    {
        servoPosition = 0.2f;
        intake.grab();
    }

    public void intake_up()
    {
        intake.stop();
        servoPosition = 1.f;
    }

    public void set_servo_position(float position)
    {
        servoPosition = position;
    }

    public void intake_delivery()
    {
        intake.deliver();
    }

    public void update() throws InterruptedException
    {
        intakeServo.setPosition(servoPosition);
        intake.update();
        telemetry.addData("LeftMotorCurrent", leftArmMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RightMotorCurrent", rightArmMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("SlideMotorCurrent", slideMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("LeftMotorSteps", leftArmMotor.getCurrentPosition());
        telemetry.addData("RightMotorSteps", rightArmMotor.getCurrentPosition());
        telemetry.addData("SlideMotorSteps", slideMotor.getCurrentPosition());
        if (leftArmMotor.isOverCurrent() || rightArmMotor.isOverCurrent())
        {
            telemetry.addData("Alert", "Arm Motor Over Current");
            leftArmMotor.setPower(0.10);
            rightArmMotor.setPower(0.0);
        }
        if (slideMotor.isOverCurrent())
        {
            telemetry.addData("Alert", "Slide Motor Over Current");
            slideMotor.setPower(0.0);
        }
    }

    public boolean is_arm_lift_finished()
    {
        return !leftArmMotor.isBusy() && !rightArmMotor.isBusy();
    }

    public boolean is_arm_extend_finished()
    {
        return !slideMotor.isBusy();
    }
}
