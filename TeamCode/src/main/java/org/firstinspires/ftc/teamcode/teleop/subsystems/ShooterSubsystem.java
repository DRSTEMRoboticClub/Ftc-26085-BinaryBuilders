package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
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

    // Simple PID controller: error = TX degrees from LL, setpoint = 0 (centred on tag).
    // Reset when the driver takes manual control so stale I/D terms don't spike on handback.
    private final PIDController turretPID = new PIDController(
            ShooterConfig.TURRET_P, ShooterConfig.TURRET_I, ShooterConfig.TURRET_D);

    private int    turretTicksAtLLUpdate  = 0;    // kept for cacheLimelightResult snapshot
    private double headingAtLLUpdate      = Double.NaN;
    private double currentRobotHeadingDeg = Double.NaN;
    private boolean turretAtLimit   = false;
    private boolean turretSearching = false;

    // One Limelight result per loop — call cacheLimelightResult() at loop start.
    // TX, distance, and visible-IDs are pre-extracted into primitives so every getter
    // is a plain field read with no list iteration or object allocation per call.
    private LLResult cachedResult = null;
    private double cachedTxDeg = Double.NaN;   // NaN = tracked tag not visible
    private double cachedDistCm = -1.0;        // -1 = tag not visible or below horizon
    private String cachedVisibleTagIds = "none";
    // Diagnostics (cached so they survive cachedResult being released each loop in TeleOp).
    private int     cachedFiducialCount = 0;   // how many AprilTags the last read saw at all
    private boolean cachedResultValid = false; // LLResult.isValid() from the last read
    private int     cachedTrackedId = -1;      // the fiducial id we actually locked onto (-1 = none)
    private String  cachedDistSource = "none"; // "3D", "TY", or "none" — where cachedDistCm came from
    private String  llPipelineStatus = "not uploaded"; // result of the init pipeline upload
    private long    lastTrackedTagMs = 0;      // when we last had a real lock (for TAG_HOLD_MS)
    private boolean tagHeld = false;           // true when current tx/dist are HELD stale, not fresh

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
                // Upload the bundled pipeline config first, so the LL always runs our known-good
                // AprilTag pipeline regardless of what was last left on the device.
                uploadBundledPipeline(hMap);
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
     * Reads the pipeline JSON bundled at TeamCode/src/main/assets/{@link ShooterConfig#LL_PIPELINE_ASSET}
     * and uploads it into slot {@link ShooterConfig#APRILTAG_PIPELINE}. Runs every init so the
     * Limelight can never drift from / lose this configuration. Best-effort: any failure leaves
     * whatever pipeline is already on the device and is recorded in llPipelineStatus.
     */
    private void uploadBundledPipeline(HardwareMap hMap) {
        try {
            java.io.InputStream is =
                    hMap.appContext.getAssets().open(ShooterConfig.LL_PIPELINE_ASSET);
            java.io.ByteArrayOutputStream out = new java.io.ByteArrayOutputStream();
            byte[] chunk = new byte[4096];
            int n;
            while ((n = is.read(chunk)) > 0) out.write(chunk, 0, n);
            is.close();
            String json = out.toString("UTF-8");
            boolean ok = limelight.uploadPipeline(json, ShooterConfig.APRILTAG_PIPELINE);
            llPipelineStatus = ok ? ("uploaded " + ShooterConfig.LL_PIPELINE_ASSET)
                                  : "uploadPipeline returned false";
        } catch (Throwable t) {
            llPipelineStatus = "pipeline upload failed: " + t.getMessage();
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
            extractLimelightData(cachedResult);
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
     * One pass over the fiducials in an LLResult: counts them, records validity and the visible
     * IDs, locks onto the tracked tag (or, if TRACK_ANY_TAG, the nearest tag when the tracked
     * one is absent), and pre-extracts TX + distance into primitives. All getters below are then
     * plain field reads. Everything is stored even when the tracked tag is missing, so telemetry
     * can show WHY there is no lock (no tags at all vs. wrong id vs. invalid result).
     */
    private void extractLimelightData(LLResult result) {
        long now = System.currentTimeMillis();

        // Always refresh the "what does the camera see right now" diagnostics.
        if (result == null) {
            cachedResultValid = false;
            cachedFiducialCount = 0;
            cachedVisibleTagIds = "none";
        } else {
            cachedResultValid = result.isValid();
            List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
            if (fids == null || fids.isEmpty()) {
                cachedFiducialCount = 0;
                cachedVisibleTagIds = "none";
            } else {
                cachedFiducialCount = fids.size();
                StringBuilder ids = new StringBuilder();
                LLResultTypes.FiducialResult tracked = null;
                LLResultTypes.FiducialResult largest = null;
                double largestArea = -1.0;
                for (LLResultTypes.FiducialResult f : fids) {
                    if (ids.length() > 0) ids.append(", ");
                    ids.append(f.getFiducialId());
                    if (f.getFiducialId() == ShooterConfig.TRACKED_TAG_ID) tracked = f;
                    double area = f.getTargetArea();
                    if (area > largestArea) { largestArea = area; largest = f; }
                }
                cachedVisibleTagIds = ids.toString();

                // Prefer the configured tag; fall back to nearest only if explicitly allowed.
                LLResultTypes.FiducialResult use = (tracked != null) ? tracked
                        : (ShooterConfig.TRACK_ANY_TAG ? largest : null);
                if (use != null) {
                    cachedTrackedId = use.getFiducialId();
                    cachedTxDeg = use.getTargetXDegrees();
                    cachedDistCm = distanceFromFiducial(use);
                    lastTrackedTagMs = now;
                    tagHeld = false;
                    return; // fresh lock — done
                }
            }
        }

        // No tracked tag in this frame. Rather than instantly dropping to "no tag" (which makes
        // the hood/RPM compensation flicker off), HOLD the last good tx + distance for a short
        // window. Detection naturally skips frames; this rides through that flicker.
        if (lastTrackedTagMs != 0 && (now - lastTrackedTagMs) <= ShooterConfig.TAG_HOLD_MS) {
            tagHeld = true;            // keep cachedTxDeg / cachedDistCm / cachedTrackedId as-is
        } else {
            cachedTxDeg = Double.NaN;
            cachedDistCm = -1.0;
            cachedTrackedId = -1;
            cachedDistSource = "none";
            tagHeld = false;
        }
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
            turretPID.reset(); // clear I/D so they don't spike when auto-aim resumes
            turretSearching = false;
            double power = applyCableLimit(manualPower * ShooterConfig.TURRET_POWER_SCALE);
            lastTurretPower = power;
            turretRotation.setPower(power);
            return;
        }

        // ── Simple PID auto-aim on raw TX from Limelight ─────────────────────────
        // TX is the horizontal angle (degrees) from camera centre to the tracked tag.
        // The PID drives TX → 0. When TX is within TURRET_TOLERANCE_DEG or no tag is
        // visible, the turret holds position (power = 0).
        double power = 0;
        turretSearching = false;
        turretAtLimit = false;
        if (autoAimEnabled) {
            if (!Double.isNaN(cachedTxDeg) && Math.abs(cachedTxDeg) > ShooterConfig.TURRET_TOLERANCE_DEG) {
                // Update coefficients from Dashboard each loop so live tuning takes effect.
                turretPID.setPID(ShooterConfig.TURRET_P, ShooterConfig.TURRET_I, ShooterConfig.TURRET_D);
                double output = turretPID.calculate(cachedTxDeg, 0) * ShooterConfig.TURRET_DIRECTION_SIGN;
                power = Range.clip(output, -ShooterConfig.TURRET_MAX_POWER, ShooterConfig.TURRET_MAX_POWER);
            } else {
                turretPID.reset(); // no tag or centred — clear integrator
            }
        }

        power = applyCableLimit(power);
        lastTurretPower = power;
        turretRotation.setPower(power);
    }

    /**
     * Drive the turret to a fixed angle (degrees from start) using the same PD gains as auto-aim.
     * Call every loop instead of runTurretControl() when AprilTag tracking is not needed.
     * Cable limit is respected — the turret will stop if it reaches TURRET_FLIP_ANGLE.
     */
    public void holdTurretAtAngle(double targetDeg) {
        double error = targetDeg - getTurretAngleDeg();
        if (Math.abs(error) < ShooterConfig.AUTO_AIM_DEADBAND_DEG) {
            turretRotation.setPower(0);
            return;
        }
        double norm  = Range.clip(error / ShooterConfig.CAMERA_HALF_FOV_DEG, -1.0, 1.0);
        double power = ShooterConfig.AUTO_AIM_P_GAIN * norm;
        power = Math.copySign(
                Math.max(Math.abs(power), ShooterConfig.AUTO_AIM_MIN_POWER), power);
        power = Range.clip(power, -ShooterConfig.AUTO_AIM_MAX_POWER, ShooterConfig.AUTO_AIM_MAX_POWER);
        turretRotation.setPower(applyCableLimit(power));
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

    /** Camera-to-tag distance (cm) for a single fiducial, taken ONLY from the Limelight's
     *  inbuilt 3D pose (SolvePnP). No on-robot TY/height computation — the pipeline must have
     *  3D enabled (fiducial_skip3d:0). Returns -1 if the 3D pose is unavailable. */
    private double distanceFromFiducial(LLResultTypes.FiducialResult f) {
        try {
            Pose3D camSpace = f.getTargetPoseCameraSpace();
            if (camSpace != null && camSpace.getPosition() != null) {
                double xRight = camSpace.getPosition().toUnit(DistanceUnit.CM).x; // +right (cm)
                double zFwd   = camSpace.getPosition().toUnit(DistanceUnit.CM).z; // +forward (cm)
                double d = Math.hypot(xRight, zFwd);
                if (d > 0 && !Double.isNaN(d) && !Double.isInfinite(d)) {
                    cachedDistSource = "3D";
                    return d;
                }
            }
        } catch (Throwable ignored) { }
        // 3D pose not available — do NOT fall back to TY geometry. Surface it instead so it is
        // obvious the pipeline's 3D pose is off (fiducial_skip3d) rather than silently guessing.
        cachedDistSource = "none";
        return -1;
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
        if (!limelightStarted) return "LL not started (toggled off? USB?)";
        // Built from cached primitives so it works in TeleOp, where cachedResult is released
        // each loop. Shows exactly why there may be no lock OR no distance.
        return String.format("fids=%d valid=%b ids=[%s] track=%d lock=%d dist=%.0f(%s)%s",
                cachedFiducialCount, cachedResultValid, cachedVisibleTagIds,
                ShooterConfig.TRACKED_TAG_ID, cachedTrackedId,
                cachedDistCm, cachedDistSource, tagHeld ? " HELD" : "");
    }

    /** Where the cached distance came from: "3D", "TY", or "none". */
    public String getDistanceSource() { return cachedDistSource; }

    /** Result of uploading the bundled pipeline config at init (for telemetry). */
    public String getPipelineUploadStatus() { return llPipelineStatus; }

    /** True when tx/distance are being held from a recent sighting through detection flicker. */
    public boolean isTagHeld() { return tagHeld; }

    /** Number of AprilTags the last Limelight read saw (any id). */
    public int getFiducialCount() { return cachedFiducialCount; }

    /** Whether the last LLResult reported isValid(). */
    public boolean isLastResultValid() { return cachedResultValid; }

    /** The fiducial id currently locked onto (-1 if none). */
    public int getLockedTagId() { return cachedTrackedId; }

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
