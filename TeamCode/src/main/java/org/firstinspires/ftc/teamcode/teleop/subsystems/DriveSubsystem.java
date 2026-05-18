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
        fr.setInverted(false);
        br.setInverted(false);

        drive = new MecanumDrive(fl, fr, bl, br);
        headingPID = new PIDController(DriveConfig.HEADING_P, DriveConfig.HEADING_I, DriveConfig.HEADING_D);
        
        imu = hMap.get(IMU.class, HardwareConfig.IMU_NAME);
        // Use your preferred orientation
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

    public void driveFieldCentric(double strafe, double forward, double turn) {
        updateHeading();
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

        double multiplier = isSlowMode ? DriveConfig.SLOW_MODE_SCALE : DriveConfig.DRIVE_SCALE;
        
        // Basic Mecanum move
        drive.driveFieldCentric(strafe * multiplier, forward * multiplier, turn * multiplier, currentHeading);
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
