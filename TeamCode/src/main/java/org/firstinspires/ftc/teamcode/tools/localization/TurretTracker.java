package org.firstinspires.ftc.teamcode.tools.localization;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.LocalizationConfig;
import org.firstinspires.ftc.teamcode.teleop.subsystems.ShooterSubsystem;

import java.util.List;

/**
 * Drives the turret to keep the assigned AprilTag centered, while protecting
 * the Limelight cable from over-rotation.
 *
 * Turret angle is derived purely from the TurretMotor encoder (we do NOT assume
 * the turret faces forward). The starting tick count is captured at construction
 * and treated as 0 deg.
 *
 * Wraparound: once |angle| from start exceeds {@link LocalizationConfig#TURRET_FLIP_ANGLE},
 * the tracker stops chasing the tag and unwinds the opposite direction until it
 * is back within (FLIP_ANGLE - hysteresis), then resumes normal tracking. This
 * prevents the cable from twisting past the configured limit.
 */
public class TurretTracker {

    private final int startTicks;
    private boolean unwinding = false;
    private double lastTurretAngleDeg = 0;

    public TurretTracker(ShooterSubsystem shooter) {
        this.startTicks = shooter.getTurretTicks();
    }

    /** Current turret heading relative to its starting position (degrees, CCW+). */
    public double getTurretAngleDegrees(ShooterSubsystem shooter) {
        int delta = shooter.getTurretTicks() - startTicks;
        lastTurretAngleDeg = delta * LocalizationConfig.TURRET_DEG_PER_TICK * LocalizationConfig.TURRET_ANGLE_SIGN;
        return lastTurretAngleDeg;
    }

    /** Last computed turret angle without re-reading the encoder. */
    public double getLastTurretAngleDegrees() {
        return lastTurretAngleDeg;
    }

    /**
     * Run one tracking iteration. Reads the turret angle, decides whether to
     * unwind or chase the tag, and commands turret power.
     *
     * @param shooter turret hardware owner
     * @param tagId   alliance AprilTag to track
     */
    public void update(ShooterSubsystem shooter, int tagId) {
        double angle = getTurretAngleDegrees(shooter);

        // --- Cable-protection wraparound state machine ---
        if (!unwinding && Math.abs(angle) >= LocalizationConfig.TURRET_FLIP_ANGLE) {
            unwinding = true;
        }
        if (unwinding) {
            // Drive back toward center until safely inside the limit.
            double target = LocalizationConfig.TURRET_FLIP_ANGLE - LocalizationConfig.TURRET_FLIP_HYSTERESIS;
            if (Math.abs(angle) <= target) {
                unwinding = false;
            } else {
                // Unwind opposite to the side we are pinned on.
                double dir = angle > 0 ? -1.0 : 1.0;
                shooter.setTurretPower(dir * LocalizationConfig.TURRET_UNWIND_POWER);
                return;
            }
        }

        // --- Normal proportional tag centering ---
        Double tx = getTagTx(shooter.getLimelightResult(), tagId);
        if (tx == null) {
            shooter.setTurretPower(0);
            return;
        }
        if (Math.abs(tx) <= LocalizationConfig.TURRET_TRACK_DEADBAND_DEG) {
            shooter.setTurretPower(0);
            return;
        }
        double raw = Range.clip(tx * LocalizationConfig.TURRET_TRACK_P_GAIN,
                -LocalizationConfig.TURRET_TRACK_MAX_POWER,
                LocalizationConfig.TURRET_TRACK_MAX_POWER);
        double power = Math.signum(raw) * Math.max(Math.abs(raw), LocalizationConfig.TURRET_TRACK_MIN_POWER);
        shooter.setTurretPower(power);
    }

    public boolean isUnwinding() {
        return unwinding;
    }

    /** Horizontal offset (deg) of the requested tag from camera center, or null. */
    private static Double getTagTx(LLResult result, int tagId) {
        if (result == null) return null;
        List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
        if (fids == null) return null;
        for (LLResultTypes.FiducialResult f : fids) {
            if (f.getFiducialId() == tagId) return f.getTargetXDegrees();
        }
        return null;
    }
}
