package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MecanumDrivetrain {

    static int WHEELS_FULL_VELOCITY = 1500;
    private final DcMotorEx motorFrontLeft;
    private final DcMotorEx motorFrontRight;
    private final DcMotorEx motorRearLeft;
    private final DcMotorEx motorRearRight;
    private final DistanceSensor distanceSensorLeft;
    private final DistanceSensor distanceSensorRight;

    private final Telemetry telemetry;

    private final IMU imu;

    // Current heading in radian
    private double direction;

    public MecanumDrivetrain(Telemetry the_telemetry, IMU the_imu, DcMotorEx frontleft, DcMotorEx frontright, DcMotorEx rearleft, DcMotorEx rearright, DistanceSensor leftsensor, DistanceSensor rightsensor)
    {
        motorFrontLeft = frontleft;
        motorFrontRight = frontright;
        motorRearLeft = rearleft;
        motorRearRight = rearright;
        telemetry = the_telemetry;
        distanceSensorLeft = leftsensor;
        distanceSensorRight = rightsensor;
        imu = the_imu;
        imu.resetYaw();
        direction = 0;
    }

    public void initialise()
    {
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // back right motor
        motorRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // front left motor
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // back left motor
        motorRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void turn_right(double radian)
    {
        direction += radian;
        if (direction > Math.PI)
        {
            direction -= (Math.PI * 2);
        }
    }

    public void turn_left(double radian)
    {
        direction -= radian;
        if (direction < -Math.PI)
        {
            direction += (Math.PI * 2);
        }
    }

    public void run(double x, double y, double speed)
    {
        // get yaw angle
        // lock robot heading to the reset direction
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rx = direction + yaw;
        if (rx > Math.PI)
        {
            rx -= Math.PI * 2;
        }
        else if (rx < -Math.PI)
        {
            rx += Math.PI * 2;
        }
        rx *= 2.5;
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

        final int FRONT_LEFT_FULL_VELOCITY = 2300;
        final int FRONT_RIGHT_FULL_VELOCITY = 2400;
        final int BACK_LEFT_FULL_VELOCITY = 2450;
        final int BACK_RIGHT_FULL_VELOCITY = 2550;
        final int MINIMUM_FULL_VELOCITY = Math.min(FRONT_LEFT_FULL_VELOCITY, Math.min(FRONT_RIGHT_FULL_VELOCITY, Math.min(BACK_LEFT_FULL_VELOCITY, BACK_RIGHT_FULL_VELOCITY)));

        // scaling
        double scaledFrontLeftPower = frontLeftPower * MINIMUM_FULL_VELOCITY / FRONT_LEFT_FULL_VELOCITY * speed;
        double scaledBackLeftPower = backLeftPower * MINIMUM_FULL_VELOCITY / BACK_LEFT_FULL_VELOCITY * speed;
        double scaledFrontRightPower = frontRightPower * MINIMUM_FULL_VELOCITY / FRONT_RIGHT_FULL_VELOCITY * speed;
        double scaledBackRightPower = backRightPower * MINIMUM_FULL_VELOCITY / BACK_RIGHT_FULL_VELOCITY * speed;

        telemetry.addData("FrontLeftWheelPower", scaledFrontLeftPower);
        telemetry.addData("FrontRightWheelPower", scaledFrontRightPower);
        telemetry.addData("BackLeftWheelPower", scaledBackLeftPower);
        telemetry.addData("BackRightWheelPower", scaledBackRightPower);

        motorFrontLeft.setPower(scaledFrontLeftPower);
        motorRearLeft.setPower(scaledBackLeftPower);
        motorFrontRight.setPower(scaledFrontRightPower);
        motorRearRight.setPower(scaledBackRightPower);
/*
        telemetry.addData("FrontLeftWheelVelocity", motorFrontLeft.getVelocity());
        telemetry.addData("FrontRightWheelVelocity", motorFrontRight.getVelocity());
        telemetry.addData("BackLeftWheelVelocity", motorRearLeft.getVelocity());
        telemetry.addData("BackRightWheelVelocity", motorRearRight.getVelocity());

 */
    }

    public void corner(double corner_distance, double tolerance)
    {
        while (true) {
            double left_distance = 1000;
            while (left_distance >= 1000) {
                left_distance = distanceSensorLeft.getDistance(DistanceUnit.MM);
            }

            double right_distance = 1000;
            while (right_distance >= 1000) {
                right_distance = distanceSensorLeft.getDistance(DistanceUnit.MM);
            }

            double x = corner_distance - left_distance;
            double y = corner_distance - right_distance;

            if (Math.abs(x) < tolerance && Math.abs(y) < tolerance)
            {
                motorFrontLeft.setPower(0.0);
                motorRearLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);
                motorRearRight.setPower(0.0);
                break;
            }

            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rx = direction + yaw;
            if (rx > Math.PI)
            {
                rx -= Math.PI * 2;
            }
            else if (rx < -Math.PI)
            {
                rx += Math.PI * 2;
            }
            rx *= 2.5;

            double joystick_angle = Math.atan2(y, x); // in radian
            double m = Math.sqrt(x * x + y * y);
            double robot_angle = joystick_angle - Math.PI / 4.0;
            x = Math.cos(robot_angle) * m;
            y = Math.sin(robot_angle) * m;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            final int FRONT_LEFT_FULL_VELOCITY = 2300;
            final int FRONT_RIGHT_FULL_VELOCITY = 2400;
            final int BACK_LEFT_FULL_VELOCITY = 2450;
            final int BACK_RIGHT_FULL_VELOCITY = 2550;
            final int MINIMUM_FULL_VELOCITY = Math.min(FRONT_LEFT_FULL_VELOCITY, Math.min(FRONT_RIGHT_FULL_VELOCITY, Math.min(BACK_LEFT_FULL_VELOCITY, BACK_RIGHT_FULL_VELOCITY)));

            // scaling
            double scaledFrontLeftPower = frontLeftPower * MINIMUM_FULL_VELOCITY / FRONT_LEFT_FULL_VELOCITY / 4.0;
            double scaledBackLeftPower = backLeftPower * MINIMUM_FULL_VELOCITY / BACK_LEFT_FULL_VELOCITY / 4.0;
            double scaledFrontRightPower = frontRightPower * MINIMUM_FULL_VELOCITY / FRONT_RIGHT_FULL_VELOCITY / 4.0;
            double scaledBackRightPower = backRightPower * MINIMUM_FULL_VELOCITY / BACK_RIGHT_FULL_VELOCITY / 4.0;

            motorFrontLeft.setPower(scaledFrontLeftPower);
            motorRearLeft.setPower(scaledBackLeftPower);
            motorFrontRight.setPower(scaledFrontRightPower);
            motorRearRight.setPower(scaledBackRightPower);
        }
    }
}
