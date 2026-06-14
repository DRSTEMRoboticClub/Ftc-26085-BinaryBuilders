package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double TURRET_POWER_SCALE = 0.4;
    public static double AUTO_AIM_P_GAIN = 0.05;         // proportional gain on TX error (degrees → power)
    public static double AUTO_AIM_DEADBAND_DEG = 1.5;    // stop correcting when tag is within this many degrees of centre
    public static double AUTO_AIM_MIN_POWER = 0.15;      // floor power to overcome BRAKE-mode stiction
    public static int TRACKED_TAG_ID = 20; // Blue alliance hub tag; Red = 24
    public static int APRILTAG_PIPELINE = 0;

    public static double STOPPER_CLOSED = 0.0;
    public static double STOPPER_OPEN = 1.0;

    // Shooter Motor PID
    public static double SHOOTER_P = 0.001;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0001;

    // Shooter Target
    public static double SHOOTER_POWER = 0.80;

    // Launcher Max Power (tunable via G2 D-Pad Up/Down for polynomial calibration)
    public static double MAX_LAUNCHER_POWER = 1.0;
    public static double LAUNCHER_POWER_INCREMENT = 0.05;

    // Velocity control ceiling. Trigger input scales 0..MAX_LAUNCHER_POWER of this RPM.
    public static double MAX_LAUNCHER_RPM = 6000.0;

    // Shooter encoder spec (GoBILDA 5000 RPM motor): 537.7 counts per revolution.
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
    // Calibration data (motorValues.md):
    //   21 cm → 3000 RPM, pitch 1.00
    //  156 cm → 3800 RPM, pitch 0.61
    //  190 cm → 4000 RPM, pitch 0.50
    //  237 cm → 4200 RPM, pitch 0.36
    //  318 cm → 4800 RPM, pitch 0.21
    //
    // Toggle via FTC Dashboard — leave false until the robot has been localizer-tuned.
    public static boolean USE_DISTANCE_COMPENSATION = false;

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
