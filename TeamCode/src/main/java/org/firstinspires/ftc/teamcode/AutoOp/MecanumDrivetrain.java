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

    static private final int vision_ball_centre = 270;

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

    public void intake(double distance, double speed) throws InterruptedException {
        double artifact_location = 0;
        drive.driveRobotCentric(0, speed, 0);
        while (artifact_location == 0)
        {
            artifact_location = camera.get_artifact_location();
            logger.addData("Location", artifact_location);
            logger.update();
            Thread.sleep(20);
        }
        drive.stop();
        drive_backward(speed, 30);

        artifact_location = camera.get_artifact_location();
        int timeout = 100;
        while (Math.abs(artifact_location - vision_ball_centre) > 20 && timeout > 0)
        {
            double error = artifact_location - vision_ball_centre;
            if (artifact_location == 0)
            {
                error = 0;
                drive.driveRobotCentric(0, 0.1, 0);
            }
            else
            {
                error /= 320;
                error *= speed;
                if (Math.abs(error) < 0.2)
                {
                    error = error / Math.abs(error) * 0.2;
                }
                else if (Math.abs(error) > speed)
                {
                    error = error / Math.abs(error) * speed;
                }
                drive.driveRobotCentric(error, 0, 0);
            }

            Thread.sleep(20);
            artifact_location = camera.get_artifact_location();
            timeout--;
        }

        if (timeout <= 0)
        {
            return;
        }
        intake.intake();

        for (int i = 0; i < distance; i += 25)
        {
            drive_backward(0.3, 25);
            if (intake.getCurrentState() != TheIntakeSystem.IntakeState.INTAKING)
            {
                drive_forward(0.8, 60);
                timeout = 500;
                while (intake.getCurrentState() != TheIntakeSystem.IntakeState.IDLE && timeout > 0) {
                    intake.Update();
                    Thread.sleep(20);
                    timeout--;
                }
                break;
            }
            for (int j = 0; j < 5; j++)
            {
                intake.Update();
                Thread.sleep(30);
            }
        }
        intake.stopIntake();
    }
}
