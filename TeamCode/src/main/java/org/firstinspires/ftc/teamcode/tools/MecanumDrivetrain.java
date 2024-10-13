package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrivetrain {

    static int WHEELS_FULL_VELOCITY = 1500;
    private final DcMotorEx motorFrontLeft;
    private final DcMotorEx motorFrontRight;
    private final DcMotorEx motorRearLeft;
    private final DcMotorEx motorRearRight;

    private final Telemetry telemetry;

    private final IMU imu;

    // Current heading in radian
    private double direction;

    public MecanumDrivetrain(Telemetry the_telemetry, IMU the_imu, DcMotorEx frontleft, DcMotorEx frontright, DcMotorEx rearleft, DcMotorEx rearright)
    {
        motorFrontLeft = frontleft;
        motorFrontRight = frontright;
        motorRearLeft = rearleft;
        motorRearRight = rearright;
        telemetry = the_telemetry;
        imu = the_imu;
        imu.resetYaw();
        direction = 0;
    }

    public void initialise()
    {
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // back right motor
        motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // front left motor
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // back left motor
        motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void turn_right(double radian)
    {
        direction += radian;
        if (direction >= Math.PI)
        {
            direction -= Math.PI;
        }
    }

    public void turn_left(double radian)
    {
        direction -= radian;
        if (direction <= -Math.PI)
        {
            direction += Math.PI;
        }
    }

    public void run(double x, double y)
    {
        // get yaw angle
        // lock robot heading to the reset direction
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rx = 5.0 * (direction + yaw);
        telemetry.addData("Heading", Math.toDegrees(yaw));
        telemetry.addData("Target Heading", Math.toDegrees(direction));

        double joystick_angle = Math.atan2(y, x); // in radian
        double m = Math.sqrt(x * x + y * y);
        double robot_angle = joystick_angle - yaw;
        x = Math.cos(robot_angle) * m;
        y = Math.sin(robot_angle) * m;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // scaling
        double scaledFrontLeftPower = frontLeftPower * frontLeftPower * frontLeftPower;
        double scaledBackLeftPower = backLeftPower * backLeftPower * backLeftPower;
        double scaledFrontRightPower = frontRightPower * frontRightPower * frontRightPower;
        double scaledBackRightPower = backRightPower * backRightPower * backRightPower;

        motorFrontLeft.setVelocity(scaledFrontLeftPower*WHEELS_FULL_VELOCITY);
        motorRearLeft.setVelocity(scaledBackLeftPower*WHEELS_FULL_VELOCITY);
        motorFrontRight.setVelocity(scaledFrontRightPower*WHEELS_FULL_VELOCITY);
        motorRearRight.setVelocity(scaledBackRightPower*WHEELS_FULL_VELOCITY);
    }
}
