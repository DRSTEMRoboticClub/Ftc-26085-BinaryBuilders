package org.firstinspires.ftc.teamcode.tools.localization;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.LocalizationConfig;
import org.firstinspires.ftc.teamcode.teleop.subsystems.ShooterSubsystem;

/**
 * Drives the turret to keep the assigned AprilTag centred, while protecting
 * the Limelight cable from over-rotation.
 *
 * Control law: PD (proportional-derivative).
 *   P term — drives toward centre proportional to tx error.
 *   D term — derivative of tx; negative when approaching centre, which reduces
 *             the output and prevents the motor from overshooting the deadband.
 *
 * Cable protection: full-rotation flip.
 *   When |angle| >= TURRET_FLIP_ANGLE the tracker enters a flip — it drives in
 *   the reversal direction continuously through 0° to the equivalent safe position
 *   on the OPPOSITE side (~340° total travel at FLIP_ANGLE=180°). The camera briefly
 *   loses the tag while crossing 0° then re-acquires from the other side, which is
 *   far better than the old "back off 20°" approach that caused hunting at the limit.
 */
public class TurretTracker {

    private final int startTicks;

    private boolean unwinding = false;
    private double flipDirection = 0;    // -1 or +1, set when a flip starts
    private double lastTurretAngleDeg = 0;

    private double lastTx = 0;           // previous tx for PD derivative term
    private double lastPower = 0;        // previous power output for slew-rate limiting

    public TurretTracker(ShooterSubsystem shooter) {
        this.startTicks = shooter.getTurretTicks();
    }

    /** Current turret heading relative to its starting position (degrees, CCW+). */
    public double getTurretAngleDegrees(ShooterSubsystem shooter) {
        int delta = shooter.getTurretTicks() - startTicks;
        lastTurretAngleDeg = delta * LocalizationConfig.TURRET_DEG_PER_TICK
                           * LocalizationConfig.TURRET_ANGLE_SIGN;
        return lastTurretAngleDeg;
    }

    /** Last computed turret angle without re-reading the encoder. */
    public double getLastTurretAngleDegrees() {
        return lastTurretAngleDeg;
    }

    /**
     * Run one tracking iteration. Reads the turret angle, decides whether to
     * flip or chase the tag, and commands turret power.
     *
     * @param shooter turret hardware owner
     * @param tx      cached tag TX from ShooterSubsystem.getTrackedTagTx() — null if not seen
     */
    public void update(ShooterSubsystem shooter, Double tx) {
        double angle = getTurretAngleDegrees(shooter);

        // ── Full-rotation flip at cable limit ────────────────────────────────
        if (!unwinding && Math.abs(angle) >= LocalizationConfig.TURRET_FLIP_ANGLE) {
            unwinding = true;
            flipDirection = angle > 0 ? -1.0 : 1.0;
            lastTx = 0;
            lastPower = 0;
        }
        if (unwinding) {
            // Drive through 0° to the safe zone on the opposite side.
            double safeAngle = LocalizationConfig.TURRET_FLIP_ANGLE
                             - LocalizationConfig.TURRET_FLIP_HYSTERESIS;
            boolean done = (flipDirection < 0 && angle <= -safeAngle)
                        || (flipDirection > 0 && angle >=  safeAngle);
            if (done) {
                unwinding = false;
                lastPower = 0;
            } else {
                double flipPower = flipDirection * LocalizationConfig.TURRET_UNWIND_POWER;
                shooter.setTurretPower(flipPower);
                lastPower = flipPower;
                return;
            }
        }

        // ── Proportional tag centering (centre-to-centre comparison) ─────────
        if (tx == null) {
            shooter.setTurretPower(0);
            lastPower = 0;
            // Preserve lastTx so derivative is smooth when the tag reappears.
            return;
        }
        if (Math.abs(tx) <= LocalizationConfig.TURRET_TRACK_DEADBAND_DEG) {
            shooter.setTurretPower(0);
            lastTx = tx;   // keep actual value (not 0) for clean derivative on next frame
            lastPower = 0;
            return;
        }

        // tx = horizontal offset between tag centre and camera centre (degrees).
        // Normalise by half-FOV so the result is -1.0 (tag at left edge of camera view)
        // to +1.0 (tag at right edge), with 0.0 meaning perfectly centred.
        double norm = Range.clip(tx / LocalizationConfig.CAMERA_HALF_FOV_DEG, -1.0, 1.0);

        // Rate of change of the normalised offset, clamped to suppress noise spikes.
        double dNorm = Range.clip((tx - lastTx) / LocalizationConfig.CAMERA_HALF_FOV_DEG,
                                  -0.5, 0.5);
        lastTx = tx;

        // P term: turret speed scales with how far off-centre the tag is.
        //   norm=0   → 0 power   (tag centred — only deadband stops it, not this)
        //   norm=±1  → ±P_GAIN power  (tag at camera edge → strongest correction)
        // D term: rate-of-change of offset — negative when converging → subtracts from
        //   P term, slowing the motor before it overshoots the deadband (damping).
        double power = (norm  * LocalizationConfig.TURRET_TRACK_P_GAIN
                      + dNorm * LocalizationConfig.TURRET_TRACK_D_GAIN)
                     * LocalizationConfig.TURRET_TRACK_DIRECTION_SIGN;

        power = Range.clip(power,
                -LocalizationConfig.TURRET_TRACK_MAX_POWER,
                 LocalizationConfig.TURRET_TRACK_MAX_POWER);

        // Minimum power floor — applied only outside the deadband so the motor doesn't
        // stall on stiction when the tag is slightly off-centre.
        if (Math.abs(power) > 0 && Math.abs(power) < LocalizationConfig.TURRET_TRACK_MIN_POWER) {
            power = Math.signum(power) * LocalizationConfig.TURRET_TRACK_MIN_POWER;
        }

        // Slew-rate limit — prevents instantaneous direction reversals from tx jumps
        // (e.g., tag reacquired after dropout on the opposite side of centre).
        double maxChange = 0.10;
        power = Range.clip(power, lastPower - maxChange, lastPower + maxChange);
        lastPower = power;

        shooter.setTurretPower(power);
    }

    public boolean isUnwinding() {
        return unwinding;
    }

}
