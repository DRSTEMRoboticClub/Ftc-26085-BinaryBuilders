package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

/**
 * Tunable constants for the auto-shoot zone feature in {@link org.firstinspires.ftc.teamcode.teleop.LocalSysBase}.
 *
 * Coordinate system: same field-frame as the Road Runner / AprilTag localizer
 * (+X forward from field centre, +Y left, origin at field centre).
 * After the robot has been localizer-corrected by at least one tag observation,
 * these coordinates correspond directly to inches on the physical field.
 *
 * All values are live-editable from FTC Dashboard without redeploying.
 * Tune them by driving the robot to each zone boundary and reading the
 * "X / Y" values from the "=== ROAD RUNNER POSE ===" telemetry section.
 */
@Config
public class ShootZoneConfig {

    // ── Zone 1 (primary shooting position) ────────────────────────────────
    // Default: roughly the mid-field area on the alliance side. Tune to match
    // the actual permitted launch region for your game.
    public static double ZONE1_X_MIN = -48.0;  // inches
    public static double ZONE1_X_MAX =  48.0;
    public static double ZONE1_Y_MIN =  15.0;
    public static double ZONE1_Y_MAX =  55.0;

    // ── Zone 2 (secondary shooting position) ──────────────────────────────
    public static double ZONE2_X_MIN = -48.0;
    public static double ZONE2_X_MAX =  48.0;
    public static double ZONE2_Y_MIN = -55.0;
    public static double ZONE2_Y_MAX = -15.0;

    // ── Auto-shoot timing ─────────────────────────────────────────────────
    /** Stopper stays open this long per ball (ms). Tune to your ball size / feed rate. */
    public static long STOPPER_OPEN_MS  = 300;
    /** Gap between shots — stopper closed (ms). Too short → jams; too long → slow rate. */
    public static long STOPPER_CLOSE_MS = 120;

    // ── RPM readiness threshold ───────────────────────────────────────────
    /** Flywheel must be within this many RPM of target before stopper will open. */
    public static double SHOOT_READY_RPM_TOLERANCE = 200.0;
}
