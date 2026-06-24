package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;
import org.firstinspires.ftc.teamcode.configs.LocalizationConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;

import java.util.List;

public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx launcherLeft;
    private final DcMotorEx launcherRight;
    private final DcMotorEx turretRotation; // ShooterMotor - rotates turret left/right
    private final Servo stopper;
    private final Limelight3A limelight;

    private boolean autoAimEnabled = true;
    private double targetShooterRpm = 0.0;
    private double lastTurretPower = 0.0;
    private int orientUpdateCounter = 0;       // rate-limits updateLimelightOrientation to 20 Hz
    private int statusUpdateCounter = 0;       // rate-limits limelight.getStatus() to ~2 Hz
    private String cachedStatusString = "OFF";
    private long lastLLUpdateMs = 0;           // timestamp of last actual LL fetch + extraction
    private boolean resultIsNew = false;       // true for one call to wasResultUpdated() after each fetch
    // When true the SDK background polling thread is running; false = stopped to save heap.
    // We restart 100 ms before each 250 ms read window and stop immediately after reading.
    // This cuts background LLResult object creation from ~40/s to ~4/s (10x less GC pressure).
    private boolean llPolling = false;
    private boolean limelightStarted = false;

    // Turret PID state (used by runTurretControl on the TeleOpBlue path).
    // Reset when the driver uses D-pad so a returning tag doesn't cause a derivative spike.
    private double turretPidIntegral = 0.0;
    private double turretPidLastNorm = 0.0;
    private long turretPidLastTimeNs = 0;
    private boolean turretPidActive = false; // false → skip derivative on next auto-aim step

    // Encoder snapshot at each LL read — lets runTurretControl estimate the real-time TX
    // between 500ms LL windows so the PID doesn't drive blind and overshoot the tag.
    private int turretTicksAtLLUpdate = 0;
    // Robot heading (deg): snapshot at each LL read, the heading frozen at the last actual
    // tag sighting, and the live heading fed in each loop. (currentHeading - headingAtTagSeen)
    // is how far the chassis has rotated since the tag was last seen — added into the turret
    // aim so it counter-rotates and holds the tag through fast chassis turns, and keeps the
    // search target moving correctly while the tag is out of view.
    private double headingAtLLUpdate = Double.NaN;
    private double headingAtTagSeen = Double.NaN;
    private double currentRobotHeadingDeg = Double.NaN;

    // ── Tag-direction memory + cable-flip state ───────────────────────────────
    // Absolute turret angle (deg from start) that would centre the tracked tag, recorded
    // every loop the tag is actually seen. When the tag leaves the camera view, the turret
    // slews back toward this remembered angle to re-find it (decelerating as it arrives),
    // instead of freezing. NaN = no tag has been seen yet.
    private double tagMemoryAngle = Double.NaN;
    // Cable protection: the aim target is clamped to ±TURRET_FLIP_ANGLE so the turret never
    // winds past the limit. turretAtLimit is true (telemetry only) when the tag's true bearing
    // is beyond the limit, i.e. the turret is pinned at the limit / swinging the long way round.
    private boolean turretAtLimit = false;
    private boolean turretSearching = false; // telemetry: true when slewing to reacquire

    // One Limelight result per loop — call cacheLimelightResult() at loop start.
    // TX, distance, and visible-IDs are pre-extracted into primitives so every getter
    // is a plain field read with no list iteration or object allocation per call.
    private LLResult cachedResult = null;
    private double cachedTxDeg = Double.NaN;   // NaN = tracked tag not visible
    private double cachedDistCm = -1.0;        // -1 = tag not visible or below horizon
    private String cachedVisibleTagIds = "none";

    // When true, runTurretControl() skips its internal auto-aim and only applies
    // manualPower. Used by LocalSysBase so TurretTracker has sole control over auto-aim
    // and the two systems don't fight over the motor in the same loop iteration.
    private boolean externalTurretControl = false;

    // Recorded at construction so getTurretAngleDeg() can give a relative angle for telemetry.
    private int turretStartTicks;

    // When true, cacheLimelightResult() keeps cachedResult alive for AprilTagLocalizer
    // (used in LocalSysBase / PedroAutoRunner). In TeleOpBlue this stays false so the
    // raw result is released immediately after primitive extraction.
    private boolean retainCachedResult = false;

    /** Call once at init to keep the raw LLResult alive for AprilTagLocalizer. Auto modes only. */
    public void setRetainCachedResult(boolean retain) {
        retainCachedResult = retain;
    }

    // When non-zero, this RPM is used by updatePID() instead of targetShooterRpm,
    // and setShooterVelocityRpm() calls are ignored. Used by the auto-shoot zone
    // feature so the PID keeps spinning even when the trigger is not held.
    private double autoShootRpmOverride = 0.0;

    // Custom flywheel PID state
    private double pidIntegral = 0.0;
    private double pidLastError = 0.0;
    private long pidLastTimeNs = 0;
    private double lastLauncherPower = 0.0;

    public ShooterSubsystem(HardwareMap hMap) {
        launcherLeft = hMap.get(DcMotorEx.class, HardwareConfig.LAUNCHER_LEFT_NAME);
        launcherRight = hMap.get(DcMotorEx.class, HardwareConfig.LAUNCHER_RIGHT_NAME);
        turretRotation = hMap.get(DcMotorEx.class, HardwareConfig.TURRET_ROTATION_NAME);
        stopper = hMap.get(Servo.class, HardwareConfig.STOPPER_NAME);

        // Motors share one shaft, so set opposite directions for matched wheel spin.
        launcherLeft.setDirection(DcMotorEx.Direction.FORWARD);
        launcherRight.setDirection(DcMotorEx.Direction.REVERSE);
        launcherLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretStartTicks = turretRotation.getCurrentPosition();

        Limelight3A ll = null;
        try {
            ll = hMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT_NAME);
        } catch (Throwable t) {
            ll = null;  // not in hardware map — run without it rather than failing init
        }
        limelight = ll;
        // Only start the Limelight when enabled. When disabled, every read below short-circuits
        // on the null/started checks, so the rest of the OpMode behaves exactly the same.
        if (limelight != null && ShooterConfig.LIMELIGHT_ENABLED) {
            try {
                limelight.pipelineSwitch(ShooterConfig.APRILTAG_PIPELINE);
                limelight.start();
                limelightStarted = true;
                llPolling = true;
            } catch (Throwable t) {
                limelightStarted = false;
            }
        }
    }

    /**
     * Fetch a fresh Limelight result once per loop.
     * Call this at the top of the OpMode loop, before any method that reads Limelight data.
     * All per-loop Limelight calls then use the cached value — no redundant USB/network polls.
     */
    /**
     * Single entry point for all Limelight data. Call every loop — internally gated to
     * 250 ms (4 Hz) so USB traffic and fiducial processing are minimal.
     *
     * On each actual fetch the fiducials list is iterated exactly once and the results
     * stored as primitives. Every getter below is therefore a plain field read with no
     * list iteration, no object allocation, and no USB access in the hot loop.
     */
    public void cacheLimelightResult() {
        if (!limelightStarted) {
            // Retry starting the LL every 2 s in case it wasn't ready on USB at init time.
            if (limelight != null && ShooterConfig.LIMELIGHT_ENABLED) {
                long now = System.currentTimeMillis();
                if (now - lastLLUpdateMs >= 2000) {
                    lastLLUpdateMs = now;
                    try {
                        limelight.pipelineSwitch(ShooterConfig.APRILTAG_PIPELINE);
                        limelight.start();
                        limelightStarted = true;
                        llPolling = true;
                    } catch (Throwable ignored) { }
                }
            }
            if (!limelightStarted) {
                cachedResult = null;
                cachedTxDeg = Double.NaN;
                cachedDistCm = -1.0;
                cachedVisibleTagIds = "none";
                return;
            }
        }

        // ── Throttled read — camera stays running continuously ───────────────────
        // The camera/pipeline runs the whole time so the AprilTag detector actually settles
        // and keeps reporting tags; we just sample getLatestResult() at a fixed rate to cap
        // object churn. (The old scheme stopped the camera and re-switched the pipeline every
        // 500 ms, which reloaded the detector constantly — the LL would flash green on a tag
        // but our brief read window kept coming back empty. That is fixed by not stopping it.)
        long now = System.currentTimeMillis();
        if (now - lastLLUpdateMs < ShooterConfig.LL_READ_INTERVAL_MS) return;
        lastLLUpdateMs = now;
        resultIsNew = true;
        try {
            cachedResult = limelight.getLatestResult();
            if (cachedResult != null) {
                Double tx = getTrackedTagTx(cachedResult, ShooterConfig.TRACKED_TAG_ID);
                cachedTxDeg = (tx != null) ? tx : Double.NaN;
                cachedDistCm = extractDistanceCm(cachedResult);
                cachedVisibleTagIds = extractVisibleTagIds(cachedResult);
            } else {
                cachedTxDeg = Double.NaN;
                cachedDistCm = -1.0;
                cachedVisibleTagIds = "none";
            }
            // Snapshot encoder position + robot heading so runTurretControl can estimate the
            // real-time TX between LL updates (turret + chassis rotation since this read).
            turretTicksAtLLUpdate = getTurretTicks();
            headingAtLLUpdate = currentRobotHeadingDeg;
            // Release the raw result reference immediately so it is GC-eligible as soon as
            // the SDK also drops its internal reference. In auto modes (retainCachedResult=true)
            // keep it alive for AprilTagLocalizer, which needs it until applyAprilTagCorrection().
            if (!retainCachedResult) cachedResult = null;
        } catch (Throwable ignored) { }
    }

    /**
     * Returns true once per 250ms fetch cycle, then resets to false.
     * Use to gate any expensive per-result processing (e.g. AprilTagLocalizer) so it
     * only runs when the cached data is actually new, not on every loop iteration.
     */
    public boolean wasResultUpdated() {
        boolean v = resultIsNew;
        resultIsNew = false;
        return v;
    }

    /** Lock the flywheel to a specific RPM, ignoring all external setShooterVelocityRpm calls. */
    public void setAutoShootRpmOverride(double rpm) {
        autoShootRpmOverride = rpm;
    }

    /** Restore normal trigger/hold-mode RPM control. */
    public void clearAutoShootRpmOverride() {
        autoShootRpmOverride = 0.0;
    }

    public void setShooterVelocityRpm(double rpm) {
        if (autoShootRpmOverride != 0.0) return;  // auto-shoot zone has control
        double maxRpm = Math.max(1.0, ShooterConfig.MAX_LAUNCHER_RPM);
        targetShooterRpm = Range.clip(rpm, -maxRpm, maxRpm);
    }

    /** Must be called every loop iteration to drive the flywheel PID. */
    public void updatePID() {
        double target = (autoShootRpmOverride != 0.0) ? autoShootRpmOverride : targetShooterRpm;
        if (target == 0.0) {
            // Ramp down at the same rate as ramp up — an abrupt cutoff on a spinning flywheel
            // generates a back-EMF spike on the power rail that drops the Expansion Hub.
            if (lastLauncherPower > 0.0) {
                double power = Math.max(0.0, lastLauncherPower - ShooterConfig.LAUNCHER_RAMP_RATE);
                lastLauncherPower = power;
                launcherLeft.setPower(power);
                launcherRight.setPower(power);
            } else {
                launcherLeft.setPower(0);
                launcherRight.setPower(0);
                pidIntegral = 0.0;
                pidLastError = 0.0;
                pidLastTimeNs = 0;
            }
            return;
        }

        double actualRpm = getShooterVelocityRpm();
        double error = target - actualRpm;

        long nowNs = System.nanoTime();
        double dt = (pidLastTimeNs == 0) ? 0.02 : (nowNs - pidLastTimeNs) / 1.0e9;
        pidLastTimeNs = nowNs;

        pidIntegral += error * dt;
        double derivative = (dt > 0) ? (error - pidLastError) / dt : 0.0;
        pidLastError = error;

        double output = ShooterConfig.SHOOTER_P * error
                + ShooterConfig.SHOOTER_I * pidIntegral
                + ShooterConfig.SHOOTER_D * derivative;

        output = Range.clip(output, 0.0, 1.0);
        // Slew-rate limit: cap power increase per loop to prevent inrush brownout on spin-up.
        output = Range.clip(output, lastLauncherPower - ShooterConfig.LAUNCHER_RAMP_RATE * 2,
                                    lastLauncherPower + ShooterConfig.LAUNCHER_RAMP_RATE);
        lastLauncherPower = output;
        launcherLeft.setPower(output);
        launcherRight.setPower(output);
    }

    public double getShooterVelocityRpm() {
        double leftRpm = ticksPerSecondToRpm(Math.abs(launcherLeft.getVelocity()));
        double rightRpm = ticksPerSecondToRpm(Math.abs(launcherRight.getVelocity()));
        return (leftRpm + rightRpm) / 2.0;
    }

    public double getTargetShooterRpm() {
        return targetShooterRpm;
    }

    /** Returns the active RPM target — override if auto-shoot is active, else manual target. */
    public double getEffectiveTargetRpm() {
        return autoShootRpmOverride != 0.0 ? autoShootRpmOverride : targetShooterRpm;
    }

    public void setTurretPower(double power) {
        turretRotation.setPower(power);
    }

    /** Raw TurretMotor encoder position (ticks). Used to derive turret angle. */
    public int getTurretTicks() {
        return turretRotation.getCurrentPosition();
    }

    /** Turret angle in degrees from the position at OpMode start. +ve = CCW per TURRET_ANGLE_SIGN. */
    public double getTurretAngleDeg() {
        return (getTurretTicks() - turretStartTicks)
                * LocalizationConfig.TURRET_DEG_PER_TICK
                * LocalizationConfig.TURRET_ANGLE_SIGN;
    }

    /** Latest Limelight result for this loop (cached by cacheLimelightResult()). */
    public LLResult getLimelightResult() {
        return cachedResult;
    }

    public void setStopperPosition(double position) {
        stopper.setPosition(position);
    }

    public void toggleAutoAim() {
        autoAimEnabled = !autoAimEnabled;
    }

    public void toggleLimelight() {
        if (limelight == null) return;
        if (limelightStarted) {
            try { limelight.stop(); } catch (Throwable ignored) { }
            limelightStarted = false;
            llPolling = false;
            cachedResult = null;
            ShooterConfig.LIMELIGHT_ENABLED = false;
        } else {
            try {
                limelight.pipelineSwitch(ShooterConfig.APRILTAG_PIPELINE);
                limelight.start();
                limelightStarted = true;
                llPolling = true;
                ShooterConfig.LIMELIGHT_ENABLED = true;
            } catch (Throwable ignored) { }
        }
    }

    public boolean isLimelightEnabled() {
        return limelightStarted;
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    /**
     * When enabled, runTurretControl() will only apply manual D-pad power — it will not
     * auto-aim internally. Set this to true in LocalSysBase so TurretTracker is the sole
     * authority on turret auto-aim and the two don't conflict in the same loop.
     */
    public void setExternalTurretControl(boolean external) {
        externalTurretControl = external;
    }

    /**
     * Feed the robot's current heading (degrees, CCW+) in once per loop, before
     * runTurretControl(). Used for the heading feed-forward that keeps the turret on the tag
     * while the chassis is rotating. Safe to omit — the turret just falls back to LL-only
     * tracking if it is never called.
     */
    public void setRobotHeading(double headingDeg) {
        currentRobotHeadingDeg = headingDeg;
    }

    /** Wrap an angle difference into (-180, 180] degrees. */
    private static double normalizeDeg(double deg) {
        while (deg >  180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    public double getLastTurretPower() {
        return lastTurretPower;
    }

    /** TX (horizontal offset, degrees) of the tracked tag from camera centre. Null if not seen. */
    public Double getTrackedTagTx() {
        return Double.isNaN(cachedTxDeg) ? null : cachedTxDeg;
    }

    /** Comma-separated list of all AprilTag IDs currently visible to the Limelight. */
    public String getVisibleTagIds() {
        if (limelight == null) return "no limelight";
        return cachedVisibleTagIds;
    }

    public void runTurretControl(double manualPower, boolean triggerActive) {
        if (externalTurretControl) {
            // TurretTracker owns auto-aim in LocalSysBase — only override on explicit D-pad.
            if (Math.abs(manualPower) > 0.01) {
                double power = applyCableLimit(manualPower * ShooterConfig.TURRET_POWER_SCALE);
                lastTurretPower = power;
                turretRotation.setPower(power);
            }
            return;
        }

        // ── Manual D-pad takes priority with instant response ────────────────────
        if (Math.abs(manualPower) > 0.01) {
            // Reset PID so the derivative doesn't spike when the tag is reacquired after
            // a manual move (stale lastNorm + large dt = huge derivative kick).
            turretPidActive = false;
            turretPidIntegral = 0.0;
            turretSearching = false;
            double power = applyCableLimit(manualPower * ShooterConfig.TURRET_POWER_SCALE);
            lastTurretPower = power;
            turretRotation.setPower(power);
            return;
        }

        // ── PID auto-aim, with heading feed-forward and search-to-reacquire ───────
        double power = 0;
        turretSearching = false;
        turretAtLimit = false;
        if (autoAimEnabled) {
            // The error fed to the PID is the tag's offset from camera-centre (degrees),
            // reconstructed live even though the LL only reports every LL_READ_INTERVAL_MS.
            // It is built from three pieces, all in the same CCW-positive degree convention:
            //
            //   error = cachedTx                         (offset at the last LL read)
            //         + (turretAngleNow - turretAtRead)  (turret rotated since the read)
            //         + (headingNow      - headingAtRead)*FF_GAIN   (CHASSIS rotated since read)
            //
            // The chassis term is the fix for "turn too fast and it stops tracking": when the
            // robot spins, the camera spins with it, so the tag races across the frame. Feeding
            // the heading change straight in lets the turret counter-rotate immediately instead
            // of waiting for the next LL frame (by which point the tag is gone).
            //
            // When the tag is currently visible we re-anchor tagMemoryAngle to it. When it is
            // NOT visible we keep driving toward that remembered direction (still heading-
            // compensated), so a tag knocked out of view by a fast turn is chased back in.
            Double error = null;
            boolean searching = false;

            if (!Double.isNaN(cachedTxDeg)) {
                // Tag visible: re-anchor the remembered robot-relative tag angle to this read,
                // and freeze the heading reference at this sighting.
                int deltaTicks = getTurretTicks() - turretTicksAtLLUpdate;
                double deltaTurret = deltaTicks
                        * LocalizationConfig.TURRET_DEG_PER_TICK
                        * LocalizationConfig.TURRET_ANGLE_SIGN;
                deltaTurret = Range.clip(deltaTurret,
                        -ShooterConfig.CAMERA_HALF_FOV_DEG, ShooterConfig.CAMERA_HALF_FOV_DEG);
                // Robot-relative turret angle that centred the tag at read time
                // ( angleAtRead - tx, where angleAtRead = angleNow - turretMotionSinceRead ).
                tagMemoryAngle = (getTurretAngleDeg() - deltaTurret) - cachedTxDeg;
                headingAtTagSeen = headingAtLLUpdate;
            }

            // How far the chassis has rotated since the tag was last actually seen. Measured
            // from the sighting (not the last read) so it stays correct during a long search.
            double headingComp = 0.0;
            if (!Double.isNaN(currentRobotHeadingDeg) && !Double.isNaN(headingAtTagSeen)) {
                headingComp = normalizeDeg(currentRobotHeadingDeg - headingAtTagSeen)
                        * LocalizationConfig.TURRET_HEADING_FF_GAIN;
            }

            if (!Double.isNaN(tagMemoryAngle)) {
                // Live aim target = remembered angle, shifted by how far the chassis has turned
                // since the sighting, wrapped to a true bearing, then CLAMPED to the cable limit.
                // Clamping makes the PID decelerate INTO the limit instead of chasing the tag
                // past it (the counter-rotation bug that wound the cable past the limit). When
                // the tag crosses the rear dead zone the wrapped bearing flips to the far side,
                // so the PID then swings the long way round — always staying within the limit.
                double lim = LocalizationConfig.TURRET_FLIP_ANGLE;
                double target = normalizeDeg(tagMemoryAngle - headingComp);
                double clamped = Range.clip(target, -lim, lim);
                turretAtLimit = (clamped != target);
                // NOTE: error is intentionally NOT wrapped — when the turret is pinned at one
                // limit and the target is at the other, the large unwrapped error drives it the
                // long way round (through 0) rather than the short way across the limit.
                error = getTurretAngleDeg() - clamped;
                searching = Double.isNaN(cachedTxDeg); // no live tag this read → reacquiring
            }

            if (error != null && Math.abs(error) > ShooterConfig.AUTO_AIM_DEADBAND_DEG) {
                double norm = Range.clip(error / ShooterConfig.CAMERA_HALF_FOV_DEG, -1.0, 1.0);

                // Searching uses P only — a clean decelerating glide back to the remembered
                // direction, no integral/derivative (which would wind up or kick on reacquire).
                double derivative = 0.0;
                if (!searching) {
                    long nowNs = System.nanoTime();
                    if (turretPidActive) {
                        double dt = Math.min((nowNs - turretPidLastTimeNs) / 1e9, 0.2);
                        if (dt > 0) {
                            derivative = (norm - turretPidLastNorm) / dt;
                            turretPidIntegral += norm * dt;
                            turretPidIntegral = Range.clip(turretPidIntegral, -1.0, 1.0);
                        }
                    }
                    turretPidLastNorm = norm;
                    turretPidLastTimeNs = nowNs;
                    turretPidActive = true;
                } else {
                    turretPidActive = false;
                    turretPidIntegral = 0.0;
                }

                double pid = (norm              * ShooterConfig.AUTO_AIM_P_GAIN
                            + turretPidIntegral * ShooterConfig.AUTO_AIM_I_GAIN
                            + derivative        * ShooterConfig.AUTO_AIM_D_GAIN)
                           * ShooterConfig.AUTO_AIM_DIRECTION_SIGN;
                power = Range.clip(pid, -ShooterConfig.AUTO_AIM_MAX_POWER, ShooterConfig.AUTO_AIM_MAX_POWER);
                if (Math.abs(power) > 0 && Math.abs(power) < ShooterConfig.AUTO_AIM_MIN_POWER) {
                    power = Math.signum(power) * ShooterConfig.AUTO_AIM_MIN_POWER;
                }
                turretSearching = searching;
            } else {
                // Centred on the tag (or on the remembered direction) — hold.
                turretPidActive = false;
                turretPidIntegral = 0.0;
            }
        }

        power = applyCableLimit(power);
        lastTurretPower = power;
        turretRotation.setPower(power);
    }

    /**
     * Hard cable-protection stop. The turret may travel only within ±TURRET_FLIP_ANGLE of its
     * start; winding past that strangles the cable. The auto-aim already clamps its aim target
     * to the limit (so it decelerates into it and never chases past), so this is a final safety
     * net — mainly for manual D-pad: it refuses any command that would drive further past the
     * limit. Returning toward the safe zone is always allowed.
     *
     * @param desired the power the controller wants to apply this loop
     * @return the power to actually send to the motor (0 if it would push past the limit)
     */
    private double applyCableLimit(double desired) {
        double angle = getTurretAngleDeg();
        double lim   = LocalizationConfig.TURRET_FLIP_ANGLE;
        // Positive power increases the turret angle (CCW per ANGLE_SIGN).
        if (angle >=  lim && desired > 0) return 0;   // at + limit, pushing further + → stop
        if (angle <= -lim && desired < 0) return 0;   // at - limit, pushing further - → stop
        return desired;
    }

    /** Horizontal distance (cm) to the tracked tag. Pre-computed in cacheLimelightResult(). */
    public double getTrackedTagDistanceCm() {
        return cachedDistCm;
    }

    /** True when the turret is slewing back toward the last-seen tag direction (tag out of view). */
    public boolean isTurretSearching() {
        return turretSearching;
    }

    /** True when the tag's bearing is beyond the cable limit — turret is pinned at the limit
     *  (or swinging the long way to the far side). */
    public boolean isTurretAtLimit() {
        return turretAtLimit;
    }

    private double extractDistanceCm(LLResult result) {
        try {
            List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
            if (fids == null) return -1;
            for (LLResultTypes.FiducialResult f : fids) {
                if (f.getFiducialId() != ShooterConfig.TRACKED_TAG_ID) continue;

                // Primary: AprilTag 3D solver distance, straight from the tag pose in camera
                // space. Robust — does not depend on hand-measured camera/tag heights or tilt,
                // and it is already computed by the pipeline (just a field read, no extra cost).
                // This is why the tag could be "clearly in view" yet show no distance: the TY
                // geometry below silently failed whenever the measured heights didn't match.
                Pose3D camSpace = f.getTargetPoseCameraSpace();
                if (camSpace != null && camSpace.getPosition() != null) {
                    double xRight = camSpace.getPosition().toUnit(DistanceUnit.CM).x; // +right (cm)
                    double zFwd   = camSpace.getPosition().toUnit(DistanceUnit.CM).z; // +forward (cm)
                    double d = Math.hypot(xRight, zFwd);
                    if (d > 0 && !Double.isNaN(d) && !Double.isInfinite(d)) return d;
                }

                // Fallback: TY geometry (needs CAMERA_HEIGHT_CM / TAG_CENTER_HEIGHT_CM / TILT).
                double ty         = f.getTargetYDegrees();
                double heightDiff = ShooterConfig.TAG_CENTER_HEIGHT_CM - ShooterConfig.CAMERA_HEIGHT_CM;
                double angleDeg   = ShooterConfig.CAMERA_TILT_DEG + ty;
                if (angleDeg <= 1.0) return -1;
                double d = heightDiff / Math.tan(Math.toRadians(angleDeg));
                return (d > 0 && !Double.isNaN(d) && !Double.isInfinite(d)) ? d : -1;
            }
        } catch (Throwable ignored) { }
        return -1;
    }

    private String extractVisibleTagIds(LLResult result) {
        List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
        if (fids == null || fids.isEmpty()) return "none";
        StringBuilder sb = new StringBuilder();
        for (LLResultTypes.FiducialResult f : fids) {
            if (sb.length() > 0) sb.append(", ");
            sb.append(f.getFiducialId());
        }
        return sb.toString();
    }

    private Double getTrackedTagTx(LLResult result, int tagId) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == tagId) return f.getTargetXDegrees();
        }
        return null;
    }

    public void switchPipeline(int pipeline) {
        if (limelight != null) limelight.pipelineSwitch(pipeline);
    }

    /**
     * Feeds the robot's IMU yaw to the Limelight for MegaTag2 pipelines.
     * Rate-limited to every 5th call (~20 Hz at 100 Hz loop rate) — the Limelight
     * USB bus does not need this at 100 Hz and excessive writes were a disconnect cause.
     */
    public void updateLimelightOrientation(double yawDegrees) {
        if (limelight == null) return;
        if (++orientUpdateCounter % 5 != 0) return;
        try {
            limelight.updateRobotOrientation(yawDegrees);
        } catch (Exception e) {
            // Ignore if Limelight is temporarily unavailable
        }
    }

    /**
     * FPS / CPU / temperature from the LL status endpoint, returned from a cache.
     * The underlying limelight.getStatus() USB call is limited to ~2 Hz so it does not
     * saturate the USB bus alongside cacheLimelightResult() in the tight loop.
     * FPS=0 means the LL is not processing frames (USB/power problem).
     */
    public String getLimelightStatus() {
        if (limelight == null || !limelightStarted) return "OFF";
        if (++statusUpdateCounter % 25 != 0) return cachedStatusString;
        try {
            LLStatus s = limelight.getStatus();
            cachedStatusString = String.format("fps=%d cpu=%.0f%% %.0fC",
                    (int) s.getFps(), s.getCpu(), s.getTemp());
        } catch (Throwable t) {
            cachedStatusString = "ERR";
        }
        return cachedStatusString;
    }

    /**
     * Single-line diagnostic string for driver-station telemetry.
     * Uses the cached result — does NOT poll the Limelight hardware every call.
     * FPS / pipeline-type diagnostics belong in LimelightHealthCheck, not the tight loop.
     */
    public String getLimelightDebugInfo() {
        if (limelight == null) return "LL=NULL (not in hardware map?)";
        if (cachedResult == null) return "result=NULL";
        return String.format("valid=%b ids=%s tx=%.1f°",
                cachedResult.isValid(), cachedVisibleTagIds,
                Double.isNaN(cachedTxDeg) ? 0.0 : cachedTxDeg);
    }

    public void stopLimelight() {
        if (limelightStarted) {
            try { limelight.stop(); } catch (Throwable ignored) { }
        }
    }

    /** Raw encoder ticks/s — average of left and right. Use to verify SHOOTER_ENCODER_EVENTS_PER_REV.
     *  Expected: rawTicksPerSec / EVENTS_PER_REV * 60 == displayed RPM. */
    public double getRawLauncherTicksPerSec() {
        return (Math.abs(launcherLeft.getVelocity()) + Math.abs(launcherRight.getVelocity())) / 2.0;
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * ShooterConfig.SHOOTER_ENCODER_EVENTS_PER_REV) / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / ShooterConfig.SHOOTER_ENCODER_EVENTS_PER_REV;
    }
}
