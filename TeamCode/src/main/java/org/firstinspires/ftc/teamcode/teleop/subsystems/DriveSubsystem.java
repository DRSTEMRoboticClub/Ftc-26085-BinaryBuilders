package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.configs.DriveConfig;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private final IMU imu;
    private final PIDController headingPID;
    private double targetHeading = 0;
    private boolean isSlowMode = false;
    private boolean isVerySlow = false;
    private double cachedHeading = 0;

    public DriveSubsystem(HardwareMap hMap) {
        Motor fl = new Motor(hMap, HardwareConfig.FL_NAME);
        Motor fr = new Motor(hMap, HardwareConfig.FR_NAME);
        Motor bl = new Motor(hMap, HardwareConfig.BL_NAME);
        Motor br = new Motor(hMap, HardwareConfig.BR_NAME);

        // Using RawPower to minimize Hub communication overhead
        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);

        // Standard Mecanum Inversion (Left side for counter-rotation)
        fl.setInverted(true);
        bl.setInverted(true);
        fr.setInverted(true);
        br.setInverted(true);

        drive = new MecanumDrive(fl, fr, bl, br);
        headingPID = new PIDController(DriveConfig.HEADING_P, DriveConfig.HEADING_I, DriveConfig.HEADING_D);
        
        imu = hMap.get(IMU.class, HardwareConfig.IMU_NAME);
        // Control hub: USB facing LEFT, sticker facing BACKWARD
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu.resetYaw();
    }

    public void updateHeading() {
        // Only read IMU if it's not currently disconnecting
        try {
            cachedHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        } catch (Exception e) {
            // Prevent crash if IMU disconnects
        }
    }

    public void setSlowMode(boolean enabled) {
        this.isSlowMode = enabled;
    }

    public void setVerySlow(boolean enabled) {
        this.isVerySlow = enabled;
    }

    public void driveFieldCentric(double strafe, double forward, double turn) {
        updateHeading();

        // Apply deadzone
        strafe = Math.abs(strafe) < DriveConfig.JOYSTICK_DEADZONE ? 0 : strafe;
        forward = Math.abs(forward) < DriveConfig.JOYSTICK_DEADZONE ? 0 : forward;
        turn = Math.abs(turn) < DriveConfig.JOYSTICK_DEADZONE ? 0 : turn;

        double currentHeading = cachedHeading;

        // If HEADING_P is 0, this logic is skipped for manual control
        if (DriveConfig.HEADING_P > 0) {
            if (Math.abs(turn) > 0.05) {
                targetHeading = currentHeading;
            } else {
                double error = normalizeAngle(targetHeading - currentHeading);
                turn = headingPID.calculate(0, error);
            }
        }

        // Determine speed multiplier
        double speedMultiplier;
        if (isVerySlow) {
            speedMultiplier = DriveConfig.VERY_SLOW_MODE_SCALE;
        } else if (isSlowMode) {
            speedMultiplier = DriveConfig.SLOW_MODE_SPEED_SCALE;
        } else {
            speedMultiplier = DriveConfig.NORMAL_SPEED_SCALE;
        }

        double turnMultiplier = speedMultiplier * DriveConfig.TURN_SCALE;

        // Basic Mecanum move
        drive.driveFieldCentric(strafe * speedMultiplier, forward * speedMultiplier, turn * turnMultiplier, currentHeading);
    }

    public double getHeading() {
        return cachedHeading;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public boolean isSlowMode() {
        return isSlowMode;
    }

    public void resetHeading() {
        imu.resetYaw();
        targetHeading = 0;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}
