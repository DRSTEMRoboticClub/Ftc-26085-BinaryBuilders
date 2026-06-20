package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

/**
 * Tunable constants for the LocalSys localization-testing TeleOps.
 *
 * Everything that depends on the physical robot (wheel size, turret gearing,
 * camera mounting, alliance tag IDs / field positions) lives here so the Blue
 * and Red OpModes stay identical and only swap a handful of values.
 *
 * Units: centimetres for distance, degrees for angles on telemetry / config,
 * radians internally inside the Road Runner localizer.
 */
@Config
public class LocalizationConfig {

    // ============================================================
    // DRIVETRAIN ODOMETRY (Road Runner mecanum dead reckoning)
    // ============================================================

    /**
     * Centimetres travelled at the wheel per encoder tick — used by Road Runner
     * MecanumLocalizer (which correctly averages all 4 encoders).
     * Theoretical: wheel_circumference_cm / CPR.  Calibrate by pushing the robot a
     * known distance and scaling until the reading matches.
     * Note: Pedro uses its own separate constant in PedroConstants because Pedro's
     * drive-encoder localizer sums (rather than averages) the 4 wheel encoders.
     */
    public static double WHEEL_IN_PER_TICK = 0.0577;  // cm/tick  (0.0227 in/tick * 2.54)

    /** Distance between left and right wheel contact points (cm). */
    public static double TRACK_WIDTH = 34.3;  // cm  (13.5 in * 2.54)

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

    /**
     * Horizontal half-FOV of the Limelight 3A (degrees).
     * Used to normalise tx to [-1, +1] so the P/D gains are independent of camera model.
     * Limelight 3A spec: ~59.6° full horizontal FOV → 29.8° half.
     */
    public static double CAMERA_HALF_FOV_DEG = 29.8;

    /**
     * Proportional gain for tag centering, in normalised-offset space.
     * A normalised offset of 1.0 means the tag is at the edge of the camera view.
     * Power at edge = P_GAIN (before MIN_POWER floor and MAX_POWER clip).
     * Equivalent to 0.025 power-per-degree in raw degree space (0.025 * 29.8 ≈ 0.745).
     */
    public static double TURRET_TRACK_P_GAIN = 0.7;

    /**
     * Derivative gain in normalised-offset space.
     * Dampens oscillation: as the turret approaches centre, the normalised offset shrinks
     * (negative dNorm), which subtracts from the P term and slows the motor before it
     * overshoots the deadband.
     * Equivalent to 0.02 power-per-degree-per-loop (0.02 * 29.8 ≈ 0.6).
     */
    public static double TURRET_TRACK_D_GAIN = 0.6;

    // Flip to +1.0 if TurretTracker moves toward the tag; -1.0 if away (tunable from Dashboard)
    public static double TURRET_TRACK_DIRECTION_SIGN = -1.0;

    /** Stop correcting when tag is within this many degrees of centre (prevents hunting). */
    public static double TURRET_TRACK_DEADBAND_DEG = 2.5;

    /**
     * Floor power to overcome BRAKE-mode stiction on the turret motor.
     * Reduced from 0.15 — with PD control, the D term handles the final approach;
     * a large floor is what caused the step-discontinuity oscillation.
     */
    public static double TURRET_TRACK_MIN_POWER = 0.08;

    /** Max |power| the auto turret tracker will command. */
    public static double TURRET_TRACK_MAX_POWER = 0.6;

    /**
     * Power used during a full-rotation flip.
     * Higher than the old "unwind" power because the flip now travels ~340° (from
     * ±FLIP_ANGLE all the way through 0° to the safe zone on the other side).
     */
    public static double TURRET_UNWIND_POWER = 0.7;

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
    public static double CAMERA_OFFSET_X = 10.2;   // cm  (4.0 in * 2.54)
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

    /** Max accepted planar distance (cm) for a tag correction to be trusted. */
    public static double TAG_MAX_TRUST_DISTANCE = 305.0;  // cm  (120 in * 2.54)

    /** Low-pass blend factor when fusing a tag correction into the RR pose
     *  (0 = ignore tag, 1 = snap fully to tag). */
    public static double TAG_CORRECTION_ALPHA = 0.25;

    // ------- Alliance-specific values (overridden per OpMode) -------
    // Defaults here are the BLUE values; LocalSysRed passes its own.

    /** Field position (cm) and the IDs are set per alliance in the OpMode. */
    public static double BLUE_TAG_FIELD_X = 0.0;
    public static double BLUE_TAG_FIELD_Y = 152.4;   // cm  (60 in * 2.54)
    public static int BLUE_TAG_ID = 20;

    public static double RED_TAG_FIELD_X = 0.0;
    public static double RED_TAG_FIELD_Y = -152.4;   // cm  (-60 in * 2.54)
    public static int RED_TAG_ID = 24;
}
