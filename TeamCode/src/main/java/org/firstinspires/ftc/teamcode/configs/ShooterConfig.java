package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double TURRET_POWER_SCALE = 0.4;

    // ── Turret auto-aim: simple PIDController on raw TX degrees ─────────────────
    // P = 0.02 → at 25° TX error output ≈ 0.5 (half power). Raise if tracking is sluggish.
    // D = 0.001 → damps overshoot as the turret approaches centre. Raise if it still overshoots.
    // Tune live from FTC Dashboard without redeploying.
    public static double CAMERA_HALF_FOV_DEG = 29.8;   // Limelight 3A horizontal half-FOV (for holdTurretAtAngle)
    public static double TURRET_P             = 0.020;
    public static double TURRET_I             = 0.000;
    public static double TURRET_D             = 0.001;
    public static double TURRET_MAX_POWER     = 0.5;   // cap — 45:1 reduction is already slow
    public static double TURRET_TOLERANCE_DEG = 3.0;   // deadzone — turret holds when TX is within this (wider = less jitter while shooting)
    // Flip to +1.0 if the turret moves AWAY from the tag instead of toward it.
    public static double TURRET_DIRECTION_SIGN = 1.0;
    // Counter-turn feed-forward: power added per degree of chassis heading change since last LL
    // frame. Keeps the turret locked on the tag while the robot is rotating between 10 Hz reads.
    // Flip to negative if the turret moves WITH the robot instead of against it.
    public static double HEADING_FF_GAIN = 0.012;
    // Keep old name as alias so holdTurretAtAngle() still compiles
    public static double AUTO_AIM_DEADBAND_DEG = 1.5;
    public static double AUTO_AIM_P_GAIN       = 0.020;
    public static double AUTO_AIM_MIN_POWER    = 0.05;
    public static double AUTO_AIM_MAX_POWER    = 0.5;
    public static int TRACKED_TAG_ID = 20; // Blue alliance hub tag; Red = 24
    public static int APRILTAG_PIPELINE = 0;

    // Limelight pipeline JSON bundled as an APK asset (TeamCode/src/main/assets/). Uploaded to
    // the Limelight at init every time the robot starts, so the LL can never drift / lose its
    // configuration. Edit the asset file to change the pipeline; rebuild to deploy it.
    public static final String LL_PIPELINE_ASSET = "AprilTags.vpr";

    // Diagnostic / fallback: when true, if TRACKED_TAG_ID is not among the visible fiducials
    // the system tracks the LARGEST (nearest) AprilTag it can see instead of reporting "no tag".
    // Flip this on from Dashboard to test an ID mismatch — if tracking suddenly works, the
    // physical tag's ID is not TRACKED_TAG_ID. Leave false for matches so it can't lock onto
    // the wrong tag (e.g. an obelisk tag).
    public static boolean TRACK_ANY_TAG = false;

    // How often (ms) cacheLimelightResult() samples the Limelight. The camera runs
    // continuously; this only rate-limits our getLatestResult() reads. 100 ms = 10 Hz,
    // plenty for tracking (the turret encoder-compensates between reads). Tune via Dashboard.
    public static long LL_READ_INTERVAL_MS = 100;

    // Tag-data hold (ms). AprilTag detection naturally flickers — a frame or two with no
    // detection between good ones. Without a hold, every dropped frame instantly resets tx and
    // distance to "no tag", so the hood/RPM compensation never settles. We instead keep the
    // last good tx + distance for this long after the tag was last seen, so brief flicker is
    // ignored. Set to 0 to disable holding. Tune via Dashboard.
    public static long TAG_HOLD_MS = 300;

    // (Distance-ramped RPM boost removed — the shooter now uses the raw polynomial RPM with
    //  no added percentage. Re-add a multiplier here only if shots come up consistently short.)

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
    // flywheel ends up spinning at 2xSHOOTER_ENCODER_EVENTS_PER_REV the commanded RPM.
    //
    // Measured empirically: a 3000 RPM command produced ~6000 real RPM with this set to 28,
    // so the true counts/rev is 14 (the hub is not 4x-quadrature decoding this encoder).
    // Re-verify with the "raw t/s" telemetry: real_RPM = raw_t/s * 60 / this value.
    public static double SHOOTER_ENCODER_CYCLES_PER_REV = 7.0;
    public static double SHOOTER_ENCODER_EVENTS_PER_REV = 28;

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
    // Calibration data (motorValues.md) — cubic fit through these 4 points:
    //   95 cm → 3900 RPM, pitch 0.145
    //  125 cm → 4200 RPM, pitch 0.085
    //  150 cm → 4400 RPM, pitch 0.025
    //  195 cm → 4900 RPM, pitch 0.000
    //
    // TRUE — the re-fit polynomial (95–195 cm data) now drives RPM + hood pitch automatically
    // whenever the goal tag is in view. The gamepad-2 manual controls still work as a fallback
    // when there is no tag. Toggle via FTC Dashboard.
    public static boolean USE_DISTANCE_COMPENSATION = true;

    // Polynomial input is clamped to this range (cm) to prevent extrapolation errors.
    // Calibration data spans 95–195 cm; clamp to it so the cubic never extrapolates.
    public static double MIN_COMP_DISTANCE = 95.0;
    public static double MAX_COMP_DISTANCE = 195.0;

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
     * Cubic through (95,3900) (125,4200) (150,4400) (195,4900):
     * rpm = 8.0808e-4·d³ − 3.3535e-1·d² + 5.4263e1·d + 1078.8
     */
    public static double hoodTuneAngle(double d) {
        return ((8.08080808e-04 * d - 3.35353535e-01) * d + 5.42626263e+01) * d
                + 1.07878788e+03;
    }

    /**
     * Hood pitch servo position [0.0 .. 1.0] for a given distance in centimetres.
     * Cubic through (95,0.145) (125,0.085) (150,0.025) (195,0.000):
     * pitch = 3.3622e-7·d³ − 1.3167e-4·d² + 1.4688e-2·d − 3.5025e-1
     */
    public static double hoodPitch(double d) {
        double p = ((3.36219336e-07 * d - 1.31673882e-04) * d + 1.46878427e-02) * d
                - 3.50254329e-01;
        return Math.max(0.0, Math.min(1.0, p));
    }
}
