package org.firstinspires.ftc.teamcode.AutoOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Tools.CameraController;
import org.firstinspires.ftc.teamcode.Tools.TheIntakeSystem;

public class MecanumDrivetrain {

    private final MecanumDrive drive;
    private final Motor frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;

    private final TheIntakeSystem intake;

    private final Telemetry logger;

    private final CameraController camera;

    static private final int vision_ball_centre = 320;

    public MecanumDrivetrain(HardwareMap hardwareMap, Telemetry telemetry, CameraController cam, TheIntakeSystem inta) {
        frontLeft  = new Motor(hardwareMap, "frontleft");
        frontRight = new Motor(hardwareMap, "frontright");
        backLeft   = new Motor(hardwareMap, "backleft");
        backRight  = new Motor(hardwareMap, "backright");
        frontRight.setDistancePerPulse(0.608);
        frontLeft.setDistancePerPulse(0.608);
        backRight.setDistancePerPulse(0.608);
        backLeft.setDistancePerPulse(0.608);
        logger = telemetry;
        camera = cam;
        intake = inta;


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
            double drive_speed = 2 *  speed / distance * (distance - distance_travelled);
            if (Math.abs(drive_speed) < 0.15)
            {
                drive_speed = drive_speed / Math.abs(drive_speed) * 0.15;
            }
            else if (Math.abs(drive_speed) > speed)
            {
                drive_speed = drive_speed / Math.abs(drive_speed) * speed;
            }
            drive.driveRobotCentric(0, drive_speed, 0);
            Thread.sleep(20);
            distance_travelled = frontRight.getDistance();
            logger.addData("Distance: ", distance_travelled);
            logger.update();
            intake.Update();
        }
        drive.stop();
    }

    public void drive_forward(double speed, double distance) throws InterruptedException {
        frontRight.resetEncoder();
        double distance_travelled = 0;
        while (distance_travelled < distance)
        {
            double drive_speed = 2 * speed / distance * (distance - distance_travelled);
            if (Math.abs(drive_speed) < 0.15)
            {
                drive_speed = drive_speed / Math.abs(drive_speed) * 0.15;
            }
            else if (Math.abs(drive_speed) > speed)
            {
                drive_speed = drive_speed / Math.abs(drive_speed) * speed;
            }
            drive.driveRobotCentric(0, -drive_speed, 0);
            Thread.sleep(20);
            distance_travelled = -frontRight.getDistance();
            logger.addData("Distance: ", distance_travelled);
            logger.update();
        }
        drive.stop();
    }

    public void turn_to(double targetAngle, double speed, double threshold) throws InterruptedException {
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = targetAngle - currentAngle;
        while (Math.abs(error) > threshold) {
            if (error > 180.0) {
                error -= 360.0;
            } else if (error < -180.0) {
                error += 360.0;
            }
            double turnSpeed = speed * Math.signum(error);
            if (Math.abs(turnSpeed) < 0.05) {
                turnSpeed = turnSpeed / Math.abs(turnSpeed) * 0.05;
            }
            else if (Math.abs(turnSpeed) > speed) {
                turnSpeed = turnSpeed / Math.abs(turnSpeed) * speed;
            }
            drive.driveRobotCentric(0, 0, -turnSpeed);
            Thread.sleep(20);
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = targetAngle + currentAngle;
        }
        drive.stop();
    }

    public void left(double speed, double distance) throws InterruptedException {
        frontRight.resetEncoder();
        distance *= Math.sqrt(2);
        distance = Math.abs(distance);
        double distance_travelled = 0;
        while (distance_travelled < distance)
        {
            double drive_speed = 2 * speed / distance * (distance - distance_travelled);
            if (Math.abs(drive_speed) < 0.1)
            {
                drive_speed = drive_speed / Math.abs(drive_speed) * 0.1;
            }
            else if (Math.abs(drive_speed) > speed)
            {
                drive_speed = drive_speed / Math.abs(drive_speed) * speed;
            }
            drive.driveRobotCentric(-drive_speed, 0, 0);
            Thread.sleep(20);
            distance_travelled = Math.abs(frontRight.getDistance());
            logger.addData("Distance: ", distance_travelled);
            logger.update();
            intake.Update();
        }
        drive.stop();
    }

    public void right(double speed, double distance) throws InterruptedException {
        left(-speed, distance);
    }

    public void stop() {
        drive.stop();
    }

    public Boolean intake(double distance, double speed) throws InterruptedException {
        intake.intake();
        camera.swithMode(CameraController.Mode.BLOB_MODE_DOWN);
        for (int i = 0; i < distance; i += 10)
        {
            double correction = camera.get_artifact_location();
            correction -= vision_ball_centre;
            correction /= 640;
            correction *= 45;
            left(speed, correction);
            drive_backward(speed, 15);
            Thread.sleep(100);
            if (intake.getCurrentState() != TheIntakeSystem.IntakeState.INTAKING)
            {
                return true;
            }
        }
        if (intake.getCurrentState() != TheIntakeSystem.IntakeState.INTAKING)
        {
            return true;
        }
        else
        {
            intake.stopIntake();
            return false;
        }
    }
}
