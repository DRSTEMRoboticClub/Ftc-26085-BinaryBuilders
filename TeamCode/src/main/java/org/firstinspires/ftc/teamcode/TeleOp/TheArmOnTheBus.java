package org.firstinspires.ftc.teamcode.TeleOp;

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

    private final Telemetry telemetry;
    private TheIntakOnTheBus intake;

    private final int MAX_SLIDE = 2150;
    private final int ARM_MOTOR_ENCODE_COUNT = (int)Math.round(288.0 / 15 * 72);

    public TheArmOnTheBus(Telemetry the_telemetry, DcMotorEx left_arm_motor, DcMotorEx right_arm_motor, DcMotorEx slide_motor, Servo intake_servo, DcMotor intake_motor, ColorRangeSensor color_sensor, String teamColour)
    {
        telemetry = the_telemetry;
        leftArmMotor = left_arm_motor;
        rightArmMotor = right_arm_motor;
        slideMotor = slide_motor;
        intakeServo = intake_servo;
        intake = new TheIntakOnTheBus(the_telemetry, intake_motor, color_sensor, teamColour);
    }

    public void initialise()
    {
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArmMotor.setTargetPositionTolerance(5);
        leftArmMotor.setCurrentAlert(4.0, CurrentUnit.AMPS);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setTargetPositionTolerance(5);
        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArmMotor.setCurrentAlert(4.0, CurrentUnit.AMPS);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setTargetPositionTolerance(10);
        slideMotor.setCurrentAlert(8.0, CurrentUnit.AMPS);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.initialise();
    }

    public void move_arm(float angle)
    {
        int targetArmPosition = Math.round(angle / 360 * ARM_MOTOR_ENCODE_COUNT);
        leftArmMotor.setTargetPosition(targetArmPosition);
        rightArmMotor.setTargetPosition(targetArmPosition);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmMotor.setPower(1.0);
        rightArmMotor.setPower(1.0);
    }

    public void expand_arm(float percentage)
    {
        int slide_target = Math.round(percentage * MAX_SLIDE);
        slideMotor.setTargetPosition(slide_target);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1.0);
    }

    public void update() throws InterruptedException
    {
        intake.update();
        telemetry.addData("LeftMotorCurrent", leftArmMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RightMotorCurrent", rightArmMotor.getCurrent(CurrentUnit.AMPS));
        /*
        if (leftArmMotor.isOverCurrent() || leftArmMotor.isOverCurrent())
        {
            telemetry.addData("Alert", "Arm Motor Over Current");
            leftArmMotor.setPower(0.0);
            rightArmMotor.setPower(0.0);
        }
        if (slideMotor.isOverCurrent())
        {
            telemetry.addData("Alert", "Slide Motor Over Current");
            slideMotor.setPower(0.0);
        }

         */
    }
}
