package org.firstinspires.ftc.teamcode.AutoOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrivetrain {

    private final MecanumDrive drive;
    private final Motor frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;

    private final Telemetry logger;

    public MecanumDrivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeft  = new Motor(hardwareMap, "frontleft");
        frontRight = new Motor(hardwareMap, "frontright");
        backLeft   = new Motor(hardwareMap, "backleft");
        backRight  = new Motor(hardwareMap, "backright");
        frontRight.setDistancePerPulse(0.608);
        frontLeft.setDistancePerPulse(0.608);
        backRight.setDistancePerPulse(0.608);
        backLeft.setDistancePerPulse(0.608);
        logger = telemetry;


        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void drive_backward(double speed, double distance) throws InterruptedException {
        frontRight.resetEncoder();
        double distance_travelled = 0;
        while (distance_travelled < distance)
        {
            double drive_speed = speed / distance * (distance - distance_travelled);
            if (drive_speed < 0.15)
            {
                drive_speed = 0.15;
            }
            else if (drive_speed > speed)
            {
                drive_speed = speed;
            }
            drive.driveRobotCentric(0, drive_speed, 0);
            Thread.sleep(100);
            distance_travelled = frontRight.getDistance();
            logger.addData("Distance: ", distance_travelled);
            logger.update();
        }
        drive.stop();
    }

    public void drive_forward(double speed, double distance) throws InterruptedException {
        frontRight.resetEncoder();
        double distance_travelled = 0;
        while (distance_travelled < distance)
        {
            double drive_speed = speed / distance * (distance - distance_travelled);
            if (drive_speed < 0.15)
            {
                drive_speed = 0.15;
            }
            else if (drive_speed > speed)
            {
                drive_speed = speed;
            }
            drive.driveRobotCentric(0, -drive_speed, 0);
            Thread.sleep(100);
            distance_travelled = -frontRight.getDistance();
            logger.addData("Distance: ", distance_travelled);
            logger.update();
        }
        drive.stop();
    }

    public void turn_to(double targetAngle, double speed, double threshold) throws InterruptedException {
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = targetAngle - currentAngle;
        while (error > threshold) {
            if (error > 180.0) {
                error -= 360.0;
            } else if (error < -180.0) {
                error += 360.0;
            }
            double turnSpeed = speed * Math.signum(error);
            if (Math.abs(turnSpeed) < 0.1) {
                turnSpeed = turnSpeed / Math.abs(turnSpeed) * 0.1;
            }
            drive.driveRobotCentric(0, 0, -turnSpeed);
            Thread.sleep(100);
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = targetAngle + currentAngle;
        }
        drive.stop();
    }

    public void stop() {
        drive.stop();
    }
}
