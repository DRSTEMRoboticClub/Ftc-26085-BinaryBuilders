package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double TURRET_POWER_SCALE = 0.4;

    // ── TeleOpBlue turret auto-aim (internal PD controller in runTurretControl) ──
    // Gains are in NORMALISED-OFFSET space: tx is divided by CAMERA_HALF_FOV_DEG before
    // multiplying by the gain, so 1.0 = tag at the very edge of the camera view.
    // This matches the LocalSys TurretTracker gains exactly so both modes behave the same.
    public static double CAMERA_HALF_FOV_DEG = 29.8;   // Limelight 3A horizontal half-FOV

    // Gains act on normalised TX: norm = tx / CAMERA_HALF_FOV_DEG → [-1, +1]
    // TX is encoder-compensated between LL updates so the PID sees live error, not stale.
    // Start here and tune P up if tracking is too sluggish, D up if it overshoots.
    public static double AUTO_AIM_P_GAIN = 0.8;
    public static double AUTO_AIM_I_GAIN = 0.0;    // raise to ~0.05 if turret consistently stops short
    public static double AUTO_AIM_D_GAIN = 0.3;    // time-based (power per normalised-error/s)
    public static double AUTO_AIM_DEADBAND_DEG = 2.5;
    public static double AUTO_AIM_MIN_POWER = 0.05;
    public static double AUTO_AIM_MAX_POWER = 0.5;

    // Step size per G2 D-pad click when tuning AUTO_AIM_P_GAIN live
    public static double AUTO_AIM_P_TUNE_STEP = 0.05;

    // Flip to +1.0 if turret moves toward the tag; -1.0 if it moves away (tunable from Dashboard)
    public static double AUTO_AIM_DIRECTION_SIGN = -1.0;
    public static int TRACKED_TAG_ID = 20; // Blue alliance hub tag; Red = 24
    public static int APRILTAG_PIPELINE = 0;

    // Multiplier applied on top of the polynomial RPM target.
    // 1.07 = 7 % boost — enough to reliably clear 3 rings; tune via FTC Dashboard.
    public static double POLY_RPM_BOOST = 1.0;

    // Distance threshold (cm) below which raw LL TX / TY measurements are used for turret
    // tracking and shooter compensation. Beyond this the localizer field position takes over:
    // dead-reckoning TX for the turret and hypot(dx,dy) for distance.
    // At 360x240 the pixel-level accuracy degrades past ~200 cm; the localizer (seeded by
    // close-range LL fixes) gives a stable, resolution-independent distance at long range.
    // Tune via FTC Dashboard — set lower if the turret overshoots at range.
    public static double LL_FALLBACK_DISTANCE_CM = 200.0;

    // DIAGNOSTIC: when false, the Limelight is never started and never read — the OpMode runs
    // identically in every other respect. Toggle from FTC Dashboard to A/B test the crash:
    //   crashes with this TRUE but NOT with it FALSE  → the Limelight (power/USB) is the cause.
    //   crashes either way                            → the cause is elsewhere.
    public static boolean LIMELIGHT_ENABLED = true;

    public static double STOPPER_CLOSED = 0.0;
    public static double STOPPER_OPEN = 1.0;

    // Shooter Motor PID
    public static double SHOOTER_P = 0.001;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0001;

    // Maximum power increase per loop iteration for the launcher motors.
    // Limits inrush current on spin-up to prevent brownouts.
    // 0.04 reaches full power in ~25 loops (~0.5s at 50 Hz). Tune via FTC Dashboard.
    public static double LAUNCHER_RAMP_RATE = 0.04;

    // Shooter Target
    public static double SHOOTER_POWER = 0.80;

    // Launcher Max Power (tunable via G2 D-Pad Up/Down for polynomial calibration)
    public static double MAX_LAUNCHER_POWER = 1.0;
    public static double LAUNCHER_POWER_INCREMENT = 0.05;

    // Velocity control ceiling. Trigger input scales 0..MAX_LAUNCHER_POWER of this RPM.
    public static double MAX_LAUNCHER_RPM = 6000.0;

    // Counts per revolution as the REV hub's getVelocity() reports them, used to convert
    // encoder ticks/s <-> RPM. This MUST match what the hub actually counts or the closed-loop
    // controller settles at the wrong speed: if this is 2x too high, the displayed RPM reads
    // half of real, the loop drives power until the (halved) reading hits target, and the
    // flywheel ends up spinning at 2x the commanded RPM.
    //
    // Measured empirically: a 3000 RPM command produced ~6000 real RPM with this set to 28,
    // so the true counts/rev is 14 (the hub is not 4x-quadrature decoding this encoder).
    // Re-verify with the "raw t/s" telemetry: real_RPM = raw_t/s * 60 / this value.
    public static double SHOOTER_ENCODER_CYCLES_PER_REV = 7.0;
    public static double SHOOTER_ENCODER_EVENTS_PER_REV = 14;

    // Manual shooter hold tuning (used from TeleOp controls)
    public static double MANUAL_TARGET_RPM = 3500.0;
    public static double RPM_TUNE_STEP_COARSE = 100.0;
    public static double RPM_TUNE_STEP_FINE = 25.0;

    // Button repeat timing for accurate tuning steps
    public static int TUNE_INITIAL_REPEAT_MS = 300;
    public static int TUNE_REPEAT_MS = 100;

    // ── Distance-based auto-compensation ──────────────────────────────────
    // When USE_DISTANCE_COMPENSATION = true, the AprilTag planar distance drives both
    // the flywheel target RPM and the hood pitch via calibrated cubic polynomials
    // (Horner form — one multiply-add per coefficient, cheap every loop).
    //
    // Calibration data (motorValues.md):
    //   21 cm → 3000 RPM, pitch 1.00
    //  156 cm → 3800 RPM, pitch 0.61
    //  190 cm → 4000 RPM, pitch 0.50
    //  237 cm → 4200 RPM, pitch 0.36
    //  318 cm → 4800 RPM, pitch 0.21
    //
    // Toggle via FTC Dashboard — leave false until the robot has been localizer-tuned.
    public static boolean USE_DISTANCE_COMPENSATION = true;

    // Polynomial input is clamped to this range (cm) to prevent extrapolation errors.
    // Calibration data spans 21–318 cm; a small margin is added on each end.
    public static double MIN_COMP_DISTANCE = 15.0;
    public static double MAX_COMP_DISTANCE = 320.0;

    // ── Camera geometry for TY-based distance (no 3D pose solver needed) ─────
    // Replaces getTargetPoseCameraSpace() with a single tan() call — orders of
    // magnitude cheaper, eliminates the LL CPU spike on tag detection.
    //
    // How to measure:
    //   CAMERA_HEIGHT_CM  — tape measure from floor to camera lens centre
    //   TAG_CENTER_HEIGHT_CM — from field spec (centre of the AprilTag face)
    //   CAMERA_TILT_DEG   — angle camera is tilted UP from horizontal; 0 = level
    //
    // All three are @Config so you can live-tune from FTC Dashboard.
    // Verify by pointing at a tag at known distance and checking "dist" in telemetry.
    public static double CAMERA_HEIGHT_CM     = 29.0;
    public static double TAG_CENTER_HEIGHT_CM = 75.0;
    public static double CAMERA_TILT_DEG      = 0.0;    // camera is level

    /**
     * Flywheel target (RPM) for a given camera-to-tag distance in centimetres.
     * tune = 6.5185e-5·d³ − 3.2190e-2·d² + 9.9130·d + 2804.79
     */
    public static double hoodTuneAngle(double d) {
        return ((6.5185466572e-05 * d - 3.2189951750e-02) * d + 9.9130248300) * d + 2804.7882679;
    }

    /**
     * Hood pitch servo position [0.0 .. 1.0] for a given distance in centimetres.
     * pitch = 2.8694e-8·d³ − 1.2812e-5·d² − 1.4227e-3·d + 1.0352
     */
    public static double hoodPitch(double d) {
        double p = ((2.8694125875e-08 * d - 1.2812102664e-05) * d - 1.4226868243e-03) * d + 1.0352425960;
        return Math.max(0.0, Math.min(1.0, p));
    }
}
