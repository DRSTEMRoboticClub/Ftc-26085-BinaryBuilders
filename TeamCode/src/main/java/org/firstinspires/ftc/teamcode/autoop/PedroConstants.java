package org.firstinspires.ftc.teamcode.autoop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configs.DriveConfig;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;
import org.firstinspires.ftc.teamcode.configs.LocalizationConfig;

/**
 * Single source of truth for the PedroPathing follower used by every auto.
 *
 * Everything Pedro needs to (a) localize and (b) drive lives here, so the auto
 * OpModes stay tiny and you only tune in ONE place. All values are {@code @Config}
 * fields, so you can live-tune them from FTC Dashboard / Panels without
 * re-deploying code.
 *
 * The localizer is Pedro's built-in DRIVE-ENCODER (mecanum dead reckoning) one:
 * it reuses the four drive motors' encoders, so it needs no extra hardware. It's
 * the quickest way to start testing paths; swap to a Pinpoint / OTOS / 3-wheel
 * localizer later by changing {@link #createFollower} if you add that hardware.
 *
 * IMPORTANT: the defaults below are STARTING POINTS. Run "Pedro Localization Test"
 * and follow https://pedropathing.com/docs/pathing tuning order:
 *   1. Localizer (ticks-to-inches + encoder directions)   <- {@link #FORWARD_TICKS_TO_INCHES} etc.
 *   2. Automatic (Forward / Lateral) tuners               <- {@link #X_VELOCITY}, {@link #Y_VELOCITY}
 *   3. PIDF tuning                                         <- {@link FollowerConstants}
 */
@Config
public class PedroConstants {

    // ============================================================
    // LOCALIZER — drive-encoder dead reckoning (TUNE FIRST)
    // ============================================================

    /**
     * Inches travelled per encoder tick, forward. Seeded from the team's existing
     * {@link LocalizationConfig#WHEEL_IN_PER_TICK}. Calibrate by pushing the robot
     * a known distance and dividing inches / ticks.
     */
    public static double FORWARD_TICKS_TO_INCHES = LocalizationConfig.WHEEL_IN_PER_TICK;
    /** Inches per tick sideways. Strafing slips, so this is usually a bit smaller. */
    public static double STRAFE_TICKS_TO_INCHES = LocalizationConfig.WHEEL_IN_PER_TICK;
    /** Inches per tick contributed to rotation. Calibrate during the turn tuner. */
    public static double TURN_TICKS_TO_INCHES = LocalizationConfig.WHEEL_IN_PER_TICK;

    /** Track width (left-right wheel distance), inches. */
    public static double ROBOT_WIDTH = LocalizationConfig.TRACK_WIDTH;
    /** Wheelbase (front-back wheel distance), inches. */
    public static double ROBOT_LENGTH = 13.5;

    /**
     * Per-wheel encoder direction (+1 / -1). During the localization test, push
     * the robot forward and flip any sign whose wheel makes X go the wrong way;
     * then strafe left and fix Y the same way.
     */
    public static double LF_ENCODER_DIR = 1.0;
    public static double RF_ENCODER_DIR = -1.0;
    public static double LR_ENCODER_DIR = 1.0;
    public static double RR_ENCODER_DIR = -1.0;

    // ============================================================
    // DRIVETRAIN — motor directions + feedforward velocities
    // ============================================================

    /**
     * Motor spin directions so that POSITIVE power drives the robot forward.
     * The team's {@code DriveSubsystem} inverts all four for FTCLib field-centric,
     * but Pedro drives the raw motors itself, so the conventional mecanum setup
     * (left side reversed) is the right starting point. Verify by commanding
     * forward in a tuner and confirming the robot goes forward, not backward.
     */
    public static DcMotorSimple.Direction LEFT_FRONT_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction LEFT_REAR_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction RIGHT_FRONT_DIR = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction RIGHT_REAR_DIR = DcMotorSimple.Direction.FORWARD;

    /** Max forward velocity (in/s) measured by Pedro's Forward Velocity Tuner. */
    public static double X_VELOCITY = 60.0;
    /** Max strafe velocity (in/s) measured by Pedro's Lateral Velocity Tuner. */
    public static double Y_VELOCITY = 50.0;
    /** Global cap on drive power (0..1). */
    public static double MAX_POWER = 1.0;

    // ============================================================
    // FOLLOWER — physics + PIDF (tune after the velocity tuners)
    // ============================================================

    /** Robot mass in kg. Used by Pedro's centripetal / braking model. */
    public static double MASS_KG = 12.0;

    /**
     * Build a fully-configured Follower from the constants above.
     * Call this once at the start of every auto / Pedro test OpMode.
     */
    public static Follower createFollower(HardwareMap hMap) {
        MecanumConstants drivetrain = new MecanumConstants()
                .leftFrontMotorName(HardwareConfig.FL_NAME)
                .rightFrontMotorName(HardwareConfig.FR_NAME)
                .leftRearMotorName(HardwareConfig.BL_NAME)
                .rightRearMotorName(HardwareConfig.BR_NAME)
                .leftFrontMotorDirection(LEFT_FRONT_DIR)
                .leftRearMotorDirection(LEFT_REAR_DIR)
                .rightFrontMotorDirection(RIGHT_FRONT_DIR)
                .rightRearMotorDirection(RIGHT_REAR_DIR)
                .xVelocity(X_VELOCITY)
                .yVelocity(Y_VELOCITY)
                .maxPower(MAX_POWER)
                .useVoltageCompensation(true)
                .nominalVoltage(DriveConfig.NOMINAL_VOLTAGE);

        DriveEncoderConstants localizer = new DriveEncoderConstants()
                .forwardTicksToInches(FORWARD_TICKS_TO_INCHES)
                .strafeTicksToInches(STRAFE_TICKS_TO_INCHES)
                .turnTicksToInches(TURN_TICKS_TO_INCHES)
                .robotWidth(ROBOT_WIDTH)
                .robotLength(ROBOT_LENGTH)
                .leftFrontEncoderDirection(LF_ENCODER_DIR)
                .rightFrontEncoderDirection(RF_ENCODER_DIR)
                .leftRearEncoderDirection(LR_ENCODER_DIR)
                .rightRearEncoderDirection(RR_ENCODER_DIR)
                .leftFrontMotorName(HardwareConfig.FL_NAME)
                .rightFrontMotorName(HardwareConfig.FR_NAME)
                .leftRearMotorName(HardwareConfig.BL_NAME)
                .rightRearMotorName(HardwareConfig.BR_NAME);

        FollowerConstants followerConstants = new FollowerConstants()
                .mass(MASS_KG);

        return new FollowerBuilder(followerConstants, hMap)
                .mecanumDrivetrain(drivetrain)
                .driveEncoderLocalizer(localizer)
                .build();
    }

    private PedroConstants() {} // static-only
}
