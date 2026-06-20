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
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;
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
    private int orientUpdateCounter = 0;  // rate-limits updateLimelightOrientation to 20 Hz
    private double lastAimTx = 0;         // previous tx for PD derivative in TeleOpBlue auto-aim
    private boolean limelightStarted = false; // true only when the LL was actually started
    // Dead-reckoning TX injected by TeleOpBlue when the LL can't see the tag.
    // Null when the LL has a real fix (real TX takes priority).
    private Double pendingFallbackTx = null;

    // One Limelight result per loop — call cacheLimelightResult() at loop start,
    // then all getters/control methods use this instead of calling getLatestResult() repeatedly.
    private LLResult cachedResult = null;

    // When true, runTurretControl() skips its internal auto-aim and only applies
    // manualPower. Used by LocalSysBase so TurretTracker has sole control over auto-aim
    // and the two systems don't fight over the motor in the same loop iteration.
    private boolean externalTurretControl = false;

    // When non-zero, this RPM is used by updatePID() instead of targetShooterRpm,
    // and setShooterVelocityRpm() calls are ignored. Used by the auto-shoot zone
    // feature so the PID keeps spinning even when the trigger is not held.
    private double autoShootRpmOverride = 0.0;

    // Custom flywheel PID state
    private double pidIntegral = 0.0;
    private double pidLastError = 0.0;
    private long pidLastTimeNs = 0;

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
    public void cacheLimelightResult() {
        if (!limelightStarted) { cachedResult = null; return; }
        try {
            cachedResult = limelight.getLatestResult();
        } catch (Throwable t) {
            // Keep last cached value if Limelight momentarily fails
        }
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
            launcherLeft.setPower(0);
            launcherRight.setPower(0);
            pidIntegral = 0.0;
            pidLastError = 0.0;
            pidLastTimeNs = 0;
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

    /** Latest Limelight result for this loop (cached by cacheLimelightResult()). */
    public LLResult getLimelightResult() {
        return cachedResult;
    }

    public void setStopperPosition(double position) {
        stopper.setPosition(position);
    }

    /** Called by TeleOpBlue each loop before runTurretControl. Null clears the fallback. */
    public void setFallbackTx(Double tx) {
        pendingFallbackTx = tx;
    }

    public void toggleAutoAim() {
        autoAimEnabled = !autoAimEnabled;
    }

    public void toggleLimelight() {
        if (limelight == null) return;
        if (limelightStarted) {
            try { limelight.stop(); } catch (Throwable ignored) { }
            limelightStarted = false;
            cachedResult = null;
            ShooterConfig.LIMELIGHT_ENABLED = false;
        } else {
            try {
                limelight.pipelineSwitch(ShooterConfig.APRILTAG_PIPELINE);
                limelight.start();
                limelightStarted = true;
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

    public double getLastTurretPower() {
        return lastTurretPower;
    }

    /** TX (horizontal offset, degrees) of the tracked tag from camera centre. Null if not seen. */
    public Double getTrackedTagTx() {
        if (cachedResult == null) return null;
        return getTrackedTagTx(cachedResult, ShooterConfig.TRACKED_TAG_ID);
    }

    /** Comma-separated list of all AprilTag IDs currently visible to the Limelight. */
    public String getVisibleTagIds() {
        if (limelight == null) return "no limelight";
        if (cachedResult == null) return "no result";
        List<LLResultTypes.FiducialResult> fids = cachedResult.getFiducialResults();
        if (fids == null || fids.isEmpty()) return "none";
        StringBuilder sb = new StringBuilder();
        for (LLResultTypes.FiducialResult f : fids) {
            if (sb.length() > 0) sb.append(", ");
            sb.append(f.getFiducialId());
        }
        return sb.toString();
    }

    public void runTurretControl(double manualPower, boolean triggerActive) {
        if (externalTurretControl) {
            // TurretTracker owns auto-aim in LocalSysBase.
            // Only override when the driver is actively pressing D-pad.
            if (Math.abs(manualPower) > 0.01) {
                double power = manualPower * ShooterConfig.TURRET_POWER_SCALE;
                lastTurretPower = power;
                turretRotation.setPower(power);
            }
            // No D-pad input → leave the motor alone so TurretTracker can write next.
            return;
        }

        // ── TeleOpBlue path: PD auto-aim (mirrors TurretTracker behaviour) ──────
        // Manual D-pad input always takes priority; auto-aim only runs when idle.
        double power = manualPower * ShooterConfig.TURRET_POWER_SCALE;

        if (autoAimEnabled && Math.abs(manualPower) < 0.01) {
            // Real TX from LL; fall back to dead-reckoning TX if tag not visible.
            Double tx = (cachedResult != null)
                    ? getTrackedTagTx(cachedResult, ShooterConfig.TRACKED_TAG_ID)
                    : null;
            if (tx == null) tx = pendingFallbackTx;

            if (tx == null) {
                // No LL fix AND no dead-reckoning estimate — hold still.
                power = 0;
            } else if (Math.abs(tx) <= ShooterConfig.AUTO_AIM_DEADBAND_DEG) {
                // Tag is centred — no correction needed.
                power = 0;
                lastAimTx = tx;
            } else {
                // Compare tag centre to camera centre and scale speed with the offset.
                // Normalise by half-FOV so the gain is independent of camera model:
                //   norm = 0   → tag at camera centre (only deadband prevents reaching here)
                //   norm = ±1  → tag at the edge of the camera view → maximum correction
                double norm  = Range.clip(tx / ShooterConfig.CAMERA_HALF_FOV_DEG, -1.0, 1.0);
                double dNorm = Range.clip((tx - lastAimTx) / ShooterConfig.CAMERA_HALF_FOV_DEG,
                                         -0.5, 0.5);
                lastAimTx = tx;

                // P: speed proportional to offset. D: damping — reduces speed as turret converges.
                double pd = (norm  * ShooterConfig.AUTO_AIM_P_GAIN
                           + dNorm * ShooterConfig.AUTO_AIM_D_GAIN)
                          * ShooterConfig.AUTO_AIM_DIRECTION_SIGN;
                power = Range.clip(pd,
                        -ShooterConfig.AUTO_AIM_MAX_POWER,
                         ShooterConfig.AUTO_AIM_MAX_POWER);

                // Min power floor so the motor overcomes stiction near the deadband edge.
                if (Math.abs(power) > 0 && Math.abs(power) < ShooterConfig.AUTO_AIM_MIN_POWER) {
                    power = Math.signum(power) * ShooterConfig.AUTO_AIM_MIN_POWER;
                }
            }
        }

        // Slew-rate limit: prevents abrupt direction reversals from causing current spikes.
        double maxChange = 0.10;
        power = Range.clip(power, lastTurretPower - maxChange, lastTurretPower + maxChange);
        lastTurretPower = power;
        turretRotation.setPower(power);
    }

    /**
     * Horizontal distance (cm) from camera to tag using TY + known geometry.
     *
     *   d = (TAG_CENTER_HEIGHT_CM - CAMERA_HEIGHT_CM) / tan(CAMERA_TILT_DEG + ty)
     *
     * This replaces the 3D pose solver entirely — one tan() call per loop.
     * No 3D pose estimation needed on the Limelight pipeline, which eliminates
     * the CPU/current spike that caused the Control Hub to brownout on detection.
     *
     * Tune CAMERA_HEIGHT_CM, TAG_CENTER_HEIGHT_CM, CAMERA_TILT_DEG in FTC Dashboard
     * and verify against a tape measure at a known distance.
     */
    public double getTrackedTagDistanceCm() {
        if (cachedResult == null) return -1;
        try {
            List<LLResultTypes.FiducialResult> fids = cachedResult.getFiducialResults();
            if (fids == null) return -1;
            for (LLResultTypes.FiducialResult f : fids) {
                if (f.getFiducialId() != ShooterConfig.TRACKED_TAG_ID) continue;
                double ty          = f.getTargetYDegrees();
                double heightDiff  = ShooterConfig.TAG_CENTER_HEIGHT_CM - ShooterConfig.CAMERA_HEIGHT_CM;
                double angleDeg    = ShooterConfig.CAMERA_TILT_DEG + ty;
                if (angleDeg <= 1.0) return -1; // tag at/below horizon — formula undefined
                double d = heightDiff / Math.tan(Math.toRadians(angleDeg));
                return (d > 0 && !Double.isNaN(d) && !Double.isInfinite(d)) ? d : -1;
            }
        } catch (Throwable t) {
            return -1;
        }
        return -1;
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
     * FPS / CPU / temperature from the LL status endpoint.
     * Called at most once per telemetry render — not called in the hot path.
     * FPS=0 means the LL is not processing frames (USB/power problem).
     */
    public String getLimelightStatus() {
        if (limelight == null || !limelightStarted) return "OFF";
        try {
            LLStatus s = limelight.getStatus();
            return String.format("fps=%d cpu=%.0f%% %.0fC", (int) s.getFps(), s.getCpu(), s.getTemp());
        } catch (Throwable t) {
            return "ERR";
        }
    }

    /**
     * Single-line diagnostic string for driver-station telemetry.
     * Uses the cached result — does NOT poll the Limelight hardware every call.
     * FPS / pipeline-type diagnostics belong in LimelightHealthCheck, not the tight loop.
     */
    public String getLimelightDebugInfo() {
        if (limelight == null) return "LL=NULL (not in hardware map?)";
        if (cachedResult == null) return "result=NULL";
        int fids = (cachedResult.getFiducialResults() != null)
                ? cachedResult.getFiducialResults().size() : -1;
        return String.format("valid=%b fids=%d tx=%.1f°",
                cachedResult.isValid(), fids, cachedResult.getTx());
    }

    public void stopLimelight() {
        if (limelightStarted) {
            try { limelight.stop(); } catch (Throwable ignored) { }
        }
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * ShooterConfig.SHOOTER_ENCODER_EVENTS_PER_REV) / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / ShooterConfig.SHOOTER_ENCODER_EVENTS_PER_REV;
    }
}
