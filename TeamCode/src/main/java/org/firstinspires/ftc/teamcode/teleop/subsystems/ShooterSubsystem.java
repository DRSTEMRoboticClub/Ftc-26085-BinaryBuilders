package org.firstinspires.ftc.teamcode.teleop.subsystems;

import android.util.Log;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    private final HardwareMap hardwareMap; // kept for re-uploading the bundled pipeline on retry

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

    private int    turretTicksAtLLUpdate  = 0;
    private double headingAtLLUpdate      = Double.NaN;
    private double currentRobotHeadingDeg = Double.NaN;
    // Full pose (Pedro field inches) — set via setRobotPose() every loop in auto modes.
    // When set, runTurretControl() compensates for translational displacement between LL reads
    // in addition to heading + turret rotation. If never set (TeleOp / heading-only callers),
    // poseX/Y stay NaN and only heading+turret compensation runs (same as before).
    private double poseX              = Double.NaN;
    private double poseY              = Double.NaN;
    private double poseXAtLLUpdate    = Double.NaN;
    private double poseYAtLLUpdate    = Double.NaN;
    private double turretDegAtLLUpdate = Double.NaN;
    private boolean turretAtLimit   = false;
    private boolean turretSearching = false;
    private boolean turretLocked    = false; // latches true once TX enters deadzone; clears only when TX exits

    // One Limelight result per loop — call cacheLimelightResult() at loop start.
    // TX, distance, and visible-IDs are pre-extracted into primitives so every getter
    // is a plain field read with no list iteration or object allocation per call.
    private LLResult cachedResult = null;
    // cachedTxDeg / cachedDistCm are corrected for TARGET_BEHIND_CM / TARGET_ABOVE_CM so
    // every downstream consumer (turret PID, distance polynomial) aims at the target point.
    private double cachedTxDeg = Double.NaN;   // corrected TX to target; NaN = tag not visible
    private double cachedDistCm = -1.0;        // corrected distance to target; -1 = not visible
    private double lastGoodDistCm = -1.0;      // last valid cachedDistCm; used when 3D pose is unavailable
    private double cachedRawTxDeg = Double.NaN; // raw LL TX to tag centre (for diagnostics)
    // Camera-space 3D components of the tag centre (cm). Set by distanceFromFiducial().
    private double cachedTagXCm = Double.NaN;  // lateral (right)
    private double cachedTagYCm = Double.NaN;  // vertical (up in camera space)
    private double cachedTagZCm = Double.NaN;  // depth (forward)
    private String cachedVisibleTagIds = "none";
    // Diagnostics (cached so they survive cachedResult being released each loop in TeleOp).
    private int     cachedFiducialCount = 0;   // how many AprilTags the last read saw at all
    private boolean cachedResultValid = false; // LLResult.isValid() from the last read
    private long    cachedStalenessMs = 0;     // LLResult.getStaleness() from the last read
    private boolean cachedResultStale = false; // true when cachedStalenessMs exceeded the threshold
    private int     cachedTrackedId = -1;      // the fiducial id we actually locked onto (-1 = none)
    private String  cachedDistSource = "none"; // "3D", "TY", or "none" — where cachedDistCm came from
    private String  llPipelineStatus = "not uploaded"; // result of the init pipeline upload
    private int     cachedLlFps  = 0;
    private double  cachedLlTempC = 0.0;
    private long    lastTrackedTagMs = 0;      // when we last had a real lock (for TAG_HOLD_MS)
    private boolean tagHeld = false;           // true when current tx/dist are HELD stale, not fresh
    // Aim bias: the PID drives estimatedTx → aimOffsetDeg instead of 0.
    // Positive = aim left of tag center (tag appears to the right in the image).
    // Set via setAimOffsetDeg(); default 0 (centre on tag).
    private double aimOffsetDeg = 0.0;

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
        hardwareMap = hMap;
        
        // ── Hardware initialization with graceful degradation ────────────────────────
        DcMotorEx ll = null, rr = null, tr = null;
        Servo st = null;
        try {
            ll = hMap.get(DcMotorEx.class, HardwareConfig.LAUNCHER_LEFT_NAME);
        } catch (Throwable t) {
            Log.e("SHOOTER", "Failed to initialize launcher left: " + t.getMessage());
        }
        try {
            rr = hMap.get(DcMotorEx.class, HardwareConfig.LAUNCHER_RIGHT_NAME);
        } catch (Throwable t) {
            Log.e("SHOOTER", "Failed to initialize launcher right: " + t.getMessage());
        }
        try {
            tr = hMap.get(DcMotorEx.class, HardwareConfig.TURRET_ROTATION_NAME);
        } catch (Throwable t) {
            Log.e("SHOOTER", "Failed to initialize turret: " + t.getMessage());
        }
        try {
            st = hMap.get(Servo.class, HardwareConfig.STOPPER_NAME);
        } catch (Throwable t) {
            Log.e("SHOOTER", "Failed to initialize stopper: " + t.getMessage());
        }
        
        launcherLeft = ll;
        launcherRight = rr;
        turretRotation = tr;
        stopper = st;

        // Motors share one shaft, so set opposite directions for matched wheel spin.
        if (launcherLeft != null) {
            try {
                launcherLeft.setDirection(DcMotorEx.Direction.FORWARD);
                launcherLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                launcherLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } catch (Throwable t) {
                Log.e("SHOOTER", "Failed to configure launcher left: " + t.getMessage());
            }
        }
        if (launcherRight != null) {
            try {
                launcherRight.setDirection(DcMotorEx.Direction.REVERSE);
                launcherRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                launcherRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } catch (Throwable t) {
                Log.e("SHOOTER", "Failed to configure launcher right: " + t.getMessage());
            }
        }
        if (turretRotation != null) {
            try {
                turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turretStartTicks = turretRotation.getCurrentPosition();
            } catch (Throwable t) {
                Log.e("SHOOTER", "Failed to configure turret: " + t.getMessage());
                turretStartTicks = 0;
            }
        } else {
            turretStartTicks = 0;
        }

        Limelight3A ll_device = null;
        try {
            ll_device = hMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT_NAME);
        } catch (Throwable t) {
            Log.w("SHOOTER", "Limelight not in hardware map: " + t.getMessage());
        }
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
            // Must re-upload the bundled pipeline here too (not just at construction) — otherwise
            // a retry after a failed/skipped init leaves the LL on whatever pipeline was already
            // on the device (e.g. a stale one from a prior session with 3D pose disabled), which
            // switches to slot 0 but never re-flashes it to our known-good AprilTags.vpr config.
            // That produces exactly "sees a tag but no distance": 2D detection still works on
            // almost any AprilTag pipeline, but 3D pose only works if fiducial_skip3d:0 is set.
            if (limelight != null && ShooterConfig.LIMELIGHT_ENABLED) {
                long now = System.currentTimeMillis();
                if (now - lastLLUpdateMs >= 2000) {
                    lastLLUpdateMs = now;
                    try {
                        uploadBundledPipeline(hardwareMap);
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
            headingAtLLUpdate     = currentRobotHeadingDeg;
            poseXAtLLUpdate       = poseX;
            poseYAtLLUpdate       = poseY;
            turretDegAtLLUpdate   = getTurretAngleDeg();
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
            cachedResultStale = false;
            cachedStalenessMs = 0;
            cachedFiducialCount = 0;
            cachedVisibleTagIds = "none";
        } else {
            cachedResultValid = result.isValid();
            // isValid() only reflects whether this JSON blob parsed correctly — NOT whether it is
            // a fresh capture. If the LL hangs, getLatestResult() can keep handing back the same
            // old (still "valid") result forever. Treat an old capture as if nothing new came in
            // so we don't report a frozen tag lock/distance as if it were live.
            cachedStalenessMs = result.getStaleness();
            cachedResultStale = cachedStalenessMs > ShooterConfig.LL_STALE_THRESHOLD_MS;
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
                // Telemetry above should still say which IDs are present in the last LLResult,
                // but stale frames must not refresh the live lock/distance used by auto-aim.
                LLResultTypes.FiducialResult use = cachedResultStale ? null
                        : (tracked != null) ? tracked
                        : (ShooterConfig.TRACK_ANY_TAG ? largest : null);
                if (use != null) {
                    cachedTrackedId = use.getFiducialId();
                    cachedRawTxDeg  = use.getTargetXDegrees();
                    cachedTxDeg     = cachedRawTxDeg;
                    cachedDistCm    = distanceFromFiducial(use); // fills cachedTagX/Y/ZCm, raw distance to tag centre

                    // Fall back to last known distance if 3D pose solver failed this frame.
                    if (cachedDistCm <= 0 && lastGoodDistCm > 0) {
                        cachedDistCm = lastGoodDistCm;
                    }

                    // Primary: full 6-DOF rotation — correct at any viewing angle.
                    // Fallback: reconstruct X/Z from raw TX + distance (no orientation data).
                    double[] target = computeTargetCameraSpace(use);
                    if (target != null) {
                        cachedTxDeg  = Math.toDegrees(Math.atan2(target[0], target[2]));
                        cachedDistCm = Math.sqrt(target[0]*target[0] + target[1]*target[1] + target[2]*target[2]);
                        lastGoodDistCm = cachedDistCm;
                    } else if (cachedDistCm > 0) {
                        double txRad = Math.toRadians(cachedRawTxDeg);
                        double tagXCm = cachedDistCm * Math.sin(txRad);
                        double tagZCm = cachedDistCm * Math.cos(txRad);
                        double tZ = tagZCm - ShooterConfig.TARGET_BEHIND_CM;
                        double tY = ShooterConfig.TARGET_ABOVE_CM;
                        cachedTxDeg  = Math.toDegrees(Math.atan2(tagXCm, tZ));
                        cachedDistCm = Math.sqrt(tagXCm*tagXCm + tZ*tZ + tY*tY);
                        lastGoodDistCm = cachedDistCm;
                    }

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
            cachedTxDeg    = Double.NaN;
            cachedRawTxDeg = Double.NaN;
            cachedDistCm   = -1.0;
            cachedTrackedId = -1;
            cachedDistSource = "none";
            cachedTagXCm = Double.NaN;
            cachedTagYCm = Double.NaN;
            cachedTagZCm = Double.NaN;
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

    public double getLeftShooterTps()  { return Math.abs(launcherLeft.getVelocity()); }
    public double getRightShooterTps() { return Math.abs(launcherRight.getVelocity()); }

    public double getLeftShooterRpm() {
        return ticksPerSecondToRpm(Math.abs(launcherLeft.getVelocity()));
    }

    public double getRightShooterRpm() {
        return ticksPerSecondToRpm(Math.abs(launcherRight.getVelocity()));
    }

    public double getShooterVelocityRpm() {
        return (getLeftShooterRpm() + getRightShooterRpm()) / 2.0;
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

    public void setAutoAimEnabled(boolean enabled) {
        autoAimEnabled = enabled;
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
                uploadBundledPipeline(hardwareMap);
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

    /**
     * Full pose update for auto modes. Enables translational displacement compensation
     * between Limelight reads in addition to heading + turret-rotation compensation.
     * Call every loop before runTurretControl(). xIn / yIn are Pedro field coordinates (inches).
     */
    public void setRobotPose(double xIn, double yIn, double headingDeg) {
        poseX = xIn;
        poseY = yIn;
        currentRobotHeadingDeg = headingDeg;
    }

    /** Aim bias applied as a PID setpoint offset. Positive = left of tag centre. */
    public void setAimOffsetDeg(double deg) { aimOffsetDeg = deg; }

    /** Wrap an angle difference into (-180, 180] degrees. */
    private static double normalizeDeg(double deg) {
        while (deg >  180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    public double getLastTurretPower() {
        return lastTurretPower;
    }

    /** Corrected TX (degrees) to the configured target point (behind/above the tag). Null if no tag. */
    public Double getTrackedTagTx() {
        return Double.isNaN(cachedTxDeg) ? null : cachedTxDeg;
    }

    /** Raw Limelight TX (degrees) to the tag centre, before target-point correction. Null if no tag. */
    public Double getRawTagTx() {
        return Double.isNaN(cachedRawTxDeg) ? null : cachedRawTxDeg;
    }

    /**
     * The pose-fused estimated TX used by runTurretControl() as the PID measurement.
     * Corrects the last LL TX for heading, turret, and translational changes since that read.
     * Returns NaN when no tag has ever been detected.
     */
    public double getEstimatedTx() {
        if (Double.isNaN(cachedTxDeg)) return Double.NaN;
        double est = cachedTxDeg;
        if (!Double.isNaN(currentRobotHeadingDeg) && !Double.isNaN(headingAtLLUpdate)) {
            est += normalizeDeg(currentRobotHeadingDeg - headingAtLLUpdate);
        }
        if (!Double.isNaN(turretDegAtLLUpdate)) {
            est -= (getTurretAngleDeg() - turretDegAtLLUpdate);
        }
        if (cachedDistCm > 0
                && !Double.isNaN(poseX) && !Double.isNaN(poseXAtLLUpdate)
                && !Double.isNaN(headingAtLLUpdate) && !Double.isNaN(turretDegAtLLUpdate)) {
            double dxCm = (poseX - poseXAtLLUpdate) * 2.54;
            double dyCm = (poseY - poseYAtLLUpdate) * 2.54;
            double tagBearingRad = Math.toRadians(
                    headingAtLLUpdate + turretDegAtLLUpdate + cachedTxDeg);
            double perp = dxCm * Math.sin(tagBearingRad) - dyCm * Math.cos(tagBearingRad);
            est -= Math.toDegrees(Math.atan2(perp, cachedDistCm));
        }
        return est;
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

        // ── Pose-fused auto-aim ───────────────────────────────────────────────────
        // Between Limelight reads (100 ms / 10 Hz) we estimate the current TX by
        // correcting the last LL reading for every source of camera motion since that read:
        //   1. Robot heading change   — 1:1 counter-rotation (replaces the old proportional FF)
        //   2. Turret rotation        — camera moved, so TX changed by the same amount
        //   3. Robot translation      — lateral/forward motion shifts the tag's apparent angle
        //      (requires setRobotPose(); gracefully skipped when only setRobotHeading() is used)
        // The estimated TX is fed directly into the PID as the measurement, so the PID reacts
        // at the full loop rate rather than waiting for the next LL frame.
        //
        // Sign notes (flip TURRET_DIRECTION_SIGN if the turret diverges instead of converges):
        //   • Heading comp: robot turns CCW (+dH) → camera also turns CCW → tag appears more
        //     to the right (TX increases) → estimatedTx += dH.
        //   • Turret comp:  turret turns CCW (+dA) → camera points more left → TX decreases
        //     → estimatedTx -= dA.
        //   • Translation:  perpendicular motion shifts the tag's apparent angle; sign is
        //     derived from the tag's bearing at the time of the last LL read.
        double power = 0;
        turretSearching = false;
        turretAtLimit = false;
        if (autoAimEnabled) {
            double estimatedTx = cachedTxDeg; // NaN when no tag has ever been seen

            if (!Double.isNaN(cachedTxDeg)) {
                // 1. Heading change since last LL update (1:1, not just a proportional gain).
                if (!Double.isNaN(currentRobotHeadingDeg) && !Double.isNaN(headingAtLLUpdate)) {
                    estimatedTx += normalizeDeg(currentRobotHeadingDeg - headingAtLLUpdate);
                }

                // 2. Turret rotation since last LL update.
                if (!Double.isNaN(turretDegAtLLUpdate)) {
                    estimatedTx -= (getTurretAngleDeg() - turretDegAtLLUpdate);
                }

                // 3. Translational displacement — needs distance and both pose snapshots.
                if (cachedDistCm > 0
                        && !Double.isNaN(poseX) && !Double.isNaN(poseXAtLLUpdate)
                        && !Double.isNaN(headingAtLLUpdate) && !Double.isNaN(turretDegAtLLUpdate)) {
                    double dxCm = (poseX - poseXAtLLUpdate) * 2.54; // inches → cm
                    double dyCm = (poseY - poseYAtLLUpdate) * 2.54;
                    // Approximate field bearing to tag at the time of the last LL read.
                    double tagBearingRad = Math.toRadians(
                            headingAtLLUpdate + turretDegAtLLUpdate + cachedTxDeg);
                    // Component of robot displacement perpendicular to the line of sight.
                    // Positive perp = robot moved so the tag appears further left → TX decreases.
                    double perp = dxCm * Math.sin(tagBearingRad) - dyCm * Math.cos(tagBearingRad);
                    estimatedTx -= Math.toDegrees(Math.atan2(perp, cachedDistCm));
                }
            }

            double txError = Double.isNaN(estimatedTx) ? Double.NaN : (estimatedTx - aimOffsetDeg);
            boolean tagOutsideDeadzone = !Double.isNaN(txError)
                    && Math.abs(txError) > ShooterConfig.TURRET_TOLERANCE_DEG;

            if (tagOutsideDeadzone) {
                turretLocked = false;
                turretPID.setPID(ShooterConfig.TURRET_P, ShooterConfig.TURRET_I, ShooterConfig.TURRET_D);
                double pidOut = turretPID.calculate(txError, 0) * ShooterConfig.TURRET_DIRECTION_SIGN;
                // Minimum power floor so static friction never stalls tracking at small angles.
                if (Math.abs(pidOut) > 0.001) {
                    pidOut = Math.copySign(
                            Math.max(Math.abs(pidOut), ShooterConfig.AUTO_AIM_MIN_POWER), pidOut);
                }
                power = Range.clip(pidOut, -ShooterConfig.TURRET_MAX_POWER, ShooterConfig.TURRET_MAX_POWER);
            } else {
                turretLocked = true;
                turretPID.reset();
                power = 0;
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

    /** True when the turret TX error is within TURRET_TOLERANCE_DEG of the aim offset — i.e. settled on target. */
    public boolean isTurretLocked() {
        return turretLocked;
    }

    /** True when the tag's bearing is beyond the cable limit — turret is pinned at the limit
     *  (or swinging the long way to the far side). */
    public boolean isTurretAtLimit() {
        return turretAtLimit;
    }

    /**
     * Returns the 3D camera-space position {x, y, z} (cm) of the aim target:
     * TARGET_BEHIND_CM behind and TARGET_ABOVE_CM above the AprilTag face.
     *
     * Uses the full 6-DOF tag orientation from SolvePnP so the offset is rotated into
     * world-correct camera-space at any viewing angle.  Plain "add to Z" is only accurate
     * when the robot faces the tag perpendicularly; this method handles side approaches.
     *
     * Tag-local frame (AprilTag standard): X right, Y up, Z out from front face toward camera.
     * "Into the goal" = −Z in tag frame.  Rotation order: ZYX Euler (yaw→pitch→roll).
     *
     * Returns null when the full pose (position + orientation) is unavailable.
     */
    private double[] computeTargetCameraSpace(LLResultTypes.FiducialResult f) {
        try {
            Pose3D pose = f.getTargetPoseCameraSpace();
            if (pose == null) {
                Log.w("SHOOTER", "computeTargetCameraSpace: Pose3D is NULL");
                return null;
            }
            
            var pos = pose.getPosition();
            var ori = pose.getOrientation();
            if (pos == null) {
                Log.w("SHOOTER", "computeTargetCameraSpace: position is NULL");
                return null;
            }
            if (ori == null) {
                Log.w("SHOOTER", "computeTargetCameraSpace: orientation is NULL");
                return null;
            }

            double tx = pos.toUnit(DistanceUnit.CM).x;
            double ty = pos.toUnit(DistanceUnit.CM).y;
            double tz = Math.abs(pos.toUnit(DistanceUnit.CM).z); // positive = in front
            if (tz < 1.0 || Double.isNaN(tz)) return null; // sanity check — tag at 0 depth or bad data

            double yaw   = Math.toRadians(ori.getYaw(AngleUnit.DEGREES));
            double pitch = Math.toRadians(ori.getPitch(AngleUnit.DEGREES));
            double roll  = Math.toRadians(ori.getRoll(AngleUnit.DEGREES));

            // Rotation matrix R (tag frame → camera frame), ZYX Euler: R = Rz(yaw)·Ry(pitch)·Rx(roll)
            double cy = Math.cos(yaw),   sy = Math.sin(yaw);
            double cp = Math.cos(pitch), sp = Math.sin(pitch);
            double cr = Math.cos(roll),  sr = Math.sin(roll);

            double r00 = cy*cp,  r01 = cy*sp*sr - sy*cr,  r02 = cy*sp*cr + sy*sr;
            double r10 = sy*cp,  r11 = sy*sp*sr + cy*cr,  r12 = sy*sp*cr - cy*sr;
            double r20 = -sp,    r21 = cp*sr,              r22 = cp*cr;

            // Target in tag-local frame: centred (x=0), ABOVE_CM up (+Y), BEHIND_CM into goal (−Z).
            double ox = 0.0;
            double oy = ShooterConfig.TARGET_ABOVE_CM;
            double oz = ShooterConfig.TARGET_BEHIND_CM; // tag +Z points toward camera; goal is −Z

            // p_camera = R · p_tag + t_tag
            double targetX = r00*ox + r01*oy + r02*oz + tx;
            double targetY = r10*ox + r11*oy + r12*oz + ty;
            double targetZ = r20*ox + r21*oy + r22*oz + tz;

            if (targetZ <= 0) return null; // target ended up behind camera — bad orientation data
            return new double[]{targetX, targetY, targetZ};
        } catch (Throwable ignored) {
            return null;
        }
    }

    /**
     * Camera-to-tag distance (cm) for a single fiducial. Prefers the Limelight's inbuilt 3D
     * pose (SolvePnP), which also populates cachedTagXCm/YCm/ZCm for target-point corrections.
     * Pipeline must have 3D enabled (fiducial_skip3d:0).
     *
     * Falls back to a simple TY/trig estimate (CAMERA_HEIGHT_CM, TAG_CENTER_HEIGHT_CM,
     * CAMERA_TILT_DEG) when the 3D pose is unavailable — e.g. the LL has no camera calibration
     * for the pipeline's current resolution, so it still reports 2D fiducial detections (tx/ty)
     * but returns a null/invalid pose. Without this, a tag the LL clearly sees produces no
     * distance at all and the polynomial never engages.
     *
     * Returns -1 if neither method produces a usable distance (clears the component fields to NaN).
     */
    private double distanceFromFiducial(LLResultTypes.FiducialResult f) {
        try {
            Pose3D camSpace = f.getTargetPoseCameraSpace();
            if (camSpace != null) {
                var pos = camSpace.getPosition();
                if (pos != null) {
                    double xRight = pos.toUnit(DistanceUnit.CM).x; // +right
                    double yUp    = pos.toUnit(DistanceUnit.CM).y; // +up
                    double zFwd   = pos.toUnit(DistanceUnit.CM).z; // +forward
                    // Camera is mounted 15° upward, so project camera Z onto the horizontal plane:
                    // floor_forward = zFwd·cos(tilt) − yUp·sin(tilt)
                    double tilt = Math.toRadians(ShooterConfig.CAMERA_TILT_DEG);
                    double hFwd = zFwd * Math.cos(tilt) - yUp * Math.sin(tilt);
                    double d = Math.hypot(xRight, hFwd);
                    if (d > 0 && !Double.isNaN(d) && !Double.isInfinite(d)) {
                        cachedTagXCm = xRight;
                        cachedTagYCm = yUp;
                        cachedTagZCm = zFwd;
                        cachedDistSource = "3D";
                        return d;
                    }
                } else {
                    Log.w("SHOOTER", "distanceFromFiducial: Pose3D.getPosition() returned NULL (missing camera calibration)");
                }
            } else {
                Log.w("SHOOTER", "distanceFromFiducial: getTargetPoseCameraSpace() returned NULL");
            }
        } catch (Throwable ignored) { }

        // 3D pose unavailable this frame — fall back to TY-based trig distance.
        try {
            double tyDeg = f.getTargetYDegrees();
            double totalAngleRad = Math.toRadians(ShooterConfig.CAMERA_TILT_DEG + tyDeg);
            double heightDeltaCm = ShooterConfig.TAG_CENTER_HEIGHT_CM - ShooterConfig.CAMERA_HEIGHT_CM;
            // Horizontal (floor-plane) distance from camera to the point under the tag:
            // heightDelta = hFwd * tan(cameraTilt + ty)  =>  hFwd = heightDelta / tan(...)
            // Defensive: check for edge cases (angle near ±90°, very small/large values)
            double tanValue = Math.tan(totalAngleRad);
            if (Math.abs(totalAngleRad) > 1e-6 && Math.abs(tanValue) > 1e-6 && !Double.isInfinite(tanValue) && !Double.isNaN(tanValue)) {
                double hFwd = heightDeltaCm / tanValue;
                double txRad = Math.toRadians(f.getTargetXDegrees());
                double cosValue = Math.cos(txRad);
                if (!Double.isNaN(cosValue) && Math.abs(cosValue) > 1e-6) {
                    double d = hFwd / cosValue;
                    if (d > 0 && !Double.isNaN(d) && !Double.isInfinite(d)) {
                        cachedTagXCm = d * Math.sin(txRad);
                        cachedTagYCm = heightDeltaCm;
                        cachedTagZCm = hFwd;
                        cachedDistSource = "TY";
                        return d;
                    } else {
                        Log.w("SHOOTER", "TY fallback: computed distance is invalid (NaN/Inf)");
                    }
                } else {
                    Log.w("SHOOTER", "TY fallback: invalid cos value or angle edge case");
                }
            } else {
                Log.w("SHOOTER", "TY fallback: invalid angle or tan value (angle too steep?)");
            }
        } catch (Throwable ignored) {
            Log.w("SHOOTER", "TY fallback exception: " + ignored.getMessage());
        }

        cachedTagXCm = Double.NaN;
        cachedTagYCm = Double.NaN;
        cachedTagZCm = Double.NaN;
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
            cachedLlFps   = (int) s.getFps();
            cachedLlTempC = s.getTemp();
            cachedStatusString = String.format("fps=%d cpu=%.0f%% %.0fC",
                    cachedLlFps, s.getCpu(), cachedLlTempC);
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
        return String.format("fids=%d valid=%b age=%dms%s ids=[%s] track=%d lock=%d rawTx=%.1f tx=%.1f dist=%.0f(%s)%s",
                cachedFiducialCount, cachedResultValid, cachedStalenessMs,
                cachedResultStale ? " STALE!" : "", cachedVisibleTagIds,
                ShooterConfig.TRACKED_TAG_ID, cachedTrackedId,
                Double.isNaN(cachedRawTxDeg) ? 0.0 : cachedRawTxDeg,
                Double.isNaN(cachedTxDeg) ? 0.0 : cachedTxDeg,
                cachedDistCm, cachedDistSource, tagHeld ? " HELD" : "");
    }

    /** Where the cached distance came from: "3D", "TY", or "none". */
    public String getDistanceSource() { return cachedDistSource; }

    /** Result of uploading the bundled pipeline config at init (for telemetry). */
    public String getPipelineUploadStatus() { return llPipelineStatus; }
    public int    getLimelightFps()   { return cachedLlFps; }
    public double getLimelightTempC() { return cachedLlTempC; }

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
