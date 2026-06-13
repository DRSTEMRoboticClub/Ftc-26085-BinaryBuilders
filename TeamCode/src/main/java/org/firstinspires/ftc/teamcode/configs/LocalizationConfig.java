package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

/**
 * Tunable constants for the LocalSys localization-testing TeleOps.
 *
 * Everything that depends on the physical robot (wheel size, turret gearing,
 * camera mounting, alliance tag IDs / field positions) lives here so the Blue
 * and Red OpModes stay identical and only swap a handful of values.
 *
 * Units: inches for distance, degrees for angles on telemetry / config,
 * radians internally inside the Road Runner localizer.
 */
@Config
public class LocalizationConfig {

    // ============================================================
    // DRIVETRAIN ODOMETRY (Road Runner mecanum dead reckoning)
    // ============================================================

    /**
     * Inches travelled at the wheel per encoder tick.
     * = (wheel circumference) / (ticks per motor rev * gear ratio).
     * MUST be calibrated by pushing the robot a known distance.
     */
    public static double WHEEL_IN_PER_TICK = 0.0227;

    /** Distance between left and right wheel contact points (inches). */
    public static double TRACK_WIDTH = 13.5;

    /**
     * Strafe (lateral) compensation. Mecanum wheels slip sideways, so measured
     * strafe travel is less than commanded. >1.0 scales strafe up. Calibrate.
     */
    public static double LATERAL_MULTIPLIER = 1.0;

    /**
     * Per-encoder direction signs. Drive motors are all inverted for driving,
     * but the raw encoder counts may not match the desired +forward / +left
     * convention. Flip these (+1 / -1) during calibration until pushing the
     * robot forward / left increases X / Y respectively.
     */
    public static double FL_TICK_SIGN = 1.0;
    public static double FR_TICK_SIGN = 1.0;
    public static double BL_TICK_SIGN = 1.0;
    public static double BR_TICK_SIGN = 1.0;

    // ============================================================
    // TURRET GEOMETRY
    // ============================================================

    /**
     * Degrees the turret rotates per TurretMotor encoder tick.
     * = 360 / (ticks per motor rev * turret gear reduction).
     * Calibrate by commanding a known rotation and reading ticks.
     */
    public static double TURRET_DEG_PER_TICK = 0.05;

    /** Sign so that a positive turret angle means turret rotated CCW (left). */
    public static double TURRET_ANGLE_SIGN = 1.0;

    /**
     * Turret cable-protection limit. When |turret angle from start| exceeds
     * this, the tracker unwinds the opposite way instead of continuing.
     */
    public static double TURRET_FLIP_ANGLE = 180.0;

    /** Hysteresis (deg) below TURRET_FLIP_ANGLE before resuming normal tracking. */
    public static double TURRET_FLIP_HYSTERESIS = 20.0;

    /** Proportional gain for centering the tag (power per degree of tx). */
    public static double TURRET_TRACK_P_GAIN = 0.02;

    /** Max |power| the auto turret tracker will command. */
    public static double TURRET_TRACK_MAX_POWER = 0.6;

    /** Power used while unwinding the turret during a flip. */
    public static double TURRET_UNWIND_POWER = 0.5;

    // ============================================================
    // CAMERA / TURRET MOUNTING OFFSETS (robot frame, inches)
    // Robot frame: +X forward, +Y left, origin at robot center / IMU.
    // ============================================================

    /** Turret pivot offset from robot center. */
    public static double TURRET_PIVOT_X = 0.0;
    public static double TURRET_PIVOT_Y = 0.0;

    /**
     * Limelight lens offset from the turret pivot, measured when the turret is
     * at its zero (forward) position. Rotated by the live turret angle at runtime.
     */
    public static double CAMERA_OFFSET_X = 4.0;
    public static double CAMERA_OFFSET_Y = 0.0;

    /**
     * Yaw of the camera optical axis relative to the turret zero direction (deg).
     * 0 = camera points the same way the turret "faces".
     */
    public static double CAMERA_YAW_OFFSET = 0.0;

    // ============================================================
    // APRILTAG
    // ============================================================

    public static int APRILTAG_PIPELINE = 0;
    public static double TAG_SIZE_IN = 6.5;

    /** Max accepted planar distance (in) for a tag correction to be trusted. */
    public static double TAG_MAX_TRUST_DISTANCE = 120.0;

    /** Low-pass blend factor when fusing a tag correction into the RR pose
     *  (0 = ignore tag, 1 = snap fully to tag). */
    public static double TAG_CORRECTION_ALPHA = 0.25;

    // ------- Alliance-specific values (overridden per OpMode) -------
    // Defaults here are the BLUE values; LocalSysRed passes its own.

    /** Field position (inches) and the IDs are set per alliance in the OpMode. */
    public static double BLUE_TAG_FIELD_X = 0.0;
    public static double BLUE_TAG_FIELD_Y = 60.0;
    public static int BLUE_TAG_ID = 20;

    public static double RED_TAG_FIELD_X = 0.0;
    public static double RED_TAG_FIELD_Y = -60.0;
    public static int RED_TAG_ID = 24;
}
