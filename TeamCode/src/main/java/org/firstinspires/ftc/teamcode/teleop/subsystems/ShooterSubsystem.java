package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;

import java.util.List;

public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx launcherLeft;
    private final DcMotorEx launcherRight;
    private final DcMotorEx turretRotation; // ShooterMotor - rotates turret left/right
    private final Servo stopper;
    private final Limelight3A limelight;
    private final PIDController shooterPID;

    private boolean autoAimEnabled = true;
    private static final double MAX_RPM = 5500.0;
    private static final double TARGET_RPM = MAX_RPM * 0.30;
    private static final double ENCODER_CPR = 28.0;

    public ShooterSubsystem(HardwareMap hMap) {
        launcherLeft = hMap.get(DcMotorEx.class, HardwareConfig.LAUNCHER_LEFT_NAME);
        launcherRight = hMap.get(DcMotorEx.class, HardwareConfig.LAUNCHER_RIGHT_NAME);
        turretRotation = hMap.get(DcMotorEx.class, HardwareConfig.TURRET_ROTATION_NAME);
        stopper = hMap.get(Servo.class, HardwareConfig.STOPPER_NAME);

        // Motors share one shaft, so set opposite directions for matched wheel spin.
        launcherLeft.setDirection(DcMotorEx.Direction.REVERSE);
        launcherRight.setDirection(DcMotorEx.Direction.FORWARD);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterPID = new PIDController(ShooterConfig.SHOOTER_P, ShooterConfig.SHOOTER_I, ShooterConfig.SHOOTER_D);
        shooterPID.setSetPoint(TARGET_RPM);

        limelight = hMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT_NAME);
        if (limelight != null) {
            limelight.pipelineSwitch(ShooterConfig.APRILTAG_PIPELINE);
            limelight.start();
        }
    }

    public void setShooterPower(double power) {
        double maxPower = Range.clip(ShooterConfig.MAX_LAUNCHER_POWER, 0.0, 1.0);
        double clipped = Range.clip(power, 0.0, maxPower);
        launcherLeft.setPower(clipped);
        launcherRight.setPower(clipped);
    }

    public double getShooterPower() {
        return (launcherLeft.getPower() + launcherRight.getPower()) / 2.0;
    }

    public void setTurretPower(double power) {
        turretRotation.setPower(power);
    }

    public void setStopperPosition(double position) {
        stopper.setPosition(position);
    }

    public void toggleAutoAim() {
        autoAimEnabled = !autoAimEnabled;
    }

    public void runTurretControl(double manualPower, boolean triggerActive) {
        double power = manualPower * ShooterConfig.TURRET_POWER_SCALE;

        if (autoAimEnabled && triggerActive && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Double tx = getTrackedTagTx(result, ShooterConfig.TRACKED_TAG_ID);
                if (tx != null) {
                    power = Range.clip(tx * ShooterConfig.AUTO_AIM_P_GAIN, -0.7, 0.7);
                }
            }
        }
        turretRotation.setPower(power);
    }

    private Double getTrackedTagTx(LLResult result, int tagId) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == tagId) return f.getTargetXDegrees();
        }
        return null;
    }

    public void stopLimelight() {
        if (limelight != null) limelight.stop();
    }
}
