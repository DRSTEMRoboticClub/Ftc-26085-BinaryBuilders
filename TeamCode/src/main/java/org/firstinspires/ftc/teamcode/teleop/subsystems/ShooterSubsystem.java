package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
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
    private final DcMotorEx shooter;
    private final DcMotorEx turret;
    private final Servo stopper;
    private final Limelight3A limelight;

    private boolean autoAimEnabled = true;

    public ShooterSubsystem(HardwareMap hMap) {
        shooter = hMap.get(DcMotorEx.class, HardwareConfig.SHOOTER_NAME);
        turret = hMap.get(DcMotorEx.class, HardwareConfig.TURRET_NAME);
        stopper = hMap.get(Servo.class, HardwareConfig.STOPPER_NAME);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT_NAME);
        if (limelight != null) {
            limelight.pipelineSwitch(ShooterConfig.APRILTAG_PIPELINE);
            limelight.start();
        }
    }

    public void setShooterPower(double power) {
        shooter.setPower(power);
    }

    public void setTurretPower(double power) {
        turret.setPower(power);
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
        turret.setPower(power);
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
