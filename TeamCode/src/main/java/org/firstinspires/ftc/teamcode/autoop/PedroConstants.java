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
     * Centimetres per encoder tick — forward.
     * Pedro's drive-encoder localizer SUMS (does not average) all 4 wheel encoders,
     * so this value is ~1/4 of the per-wheel cm/tick figure.
     * Motor: GoBilda 5203-2402-0019 (19.2:1, 312 RPM). Confirmed from product page.
     * Wheel: 104 mm diameter → circumference = π × (104/25.4) = 12.862 in.
     * Encoder: 28 PPR at motor shaft × 4 quadrature edges = 112 counts/motor rev.
     * Wheel ticks: 112 × 19.2 = 2150.4 counts/wheel rev.
     * Theoretical: 12.862 / 2150.4 = 0.005981 in/tick.
     * Empirical: physical 24 in → Pedro reported 64.81 → 24/64.81 × 0.015460 = 0.005726 in/tick.
     * Using empirical (accounts for real-world wheel compression/slip).
     * Push the robot exactly one tile (24 in) and scale if the reading is wrong.
     */
    // POSITIVE — LEFT motors are REVERSE, RIGHT are FORWARD. Encoder dirs [-1,1,-1,1] compensate
    // for motor reversal so all 4 give positive counts during forward motion. Constant must be
    // positive so forward → +X in Pedro. Negative was wrong: it caused X to decrease when going
    // forward, so Pedro never saw the robot reach target X and drove forever past the endpoint.
    public static double FORWARD_TICKS_TO_INCHES = 0.005726;  // in/tick — empirical (24 in actual, 64.81 in reported)
    // POSITIVE for same reason — strafe compensation after encoder dir reversal gives a signed
    // sum; the constant sign sets the convention. Fine-tune with a strafe push test.
    public static double STRAFE_TICKS_TO_INCHES = 0.006263;   // in/tick — forward ratio applied; verify with strafe test
    /** in/tick for rotation. Re-tune with a 360° spin test after forward/strafe are correct. */
    public static double TURN_TICKS_TO_INCHES = 0.010882;     // in/tick — old 0.029400 × same (24/64.81) correction ratio

    /** Track width (left-right wheel distance, inches). */
    public static double ROBOT_WIDTH = 13.5;   // in  (34.3 cm / 2.54)
    /** Wheelbase (front-back wheel distance, inches). */
    public static double ROBOT_LENGTH = 13.5;  // in  (34.3 cm / 2.54)

    /**
     * Per-wheel encoder direction (+1 / -1). During the localization test, push
     * the robot forward and flip any sign whose wheel makes X go the wrong way;
     * then strafe left and fix Y the same way.
     */
    // All four flipped from the previous (1,-1,1,-1) — the auto drove directly opposite to
    // its target, i.e. the localizer was reporting motion the wrong way (inverted feedback).
    // Flipping all four inverts the sensed X, Y AND heading together. These are @Config, so
    // verify/adjust live on Dashboard with "Pedro Localization Test" (push fwd -> X up, etc.).
    public static double LF_ENCODER_DIR = 1.0;
    public static double RF_ENCODER_DIR = -1.0;
    public static double LR_ENCODER_DIR = 1.0;
    public static double RR_ENCODER_DIR = -1.0;

    // ============================================================
    // DRIVETRAIN — motor directio
    // ns + feedforward velocities
    // =,./===========================================================

    /**
     * Motor spin directions so that POSITIVE power drives the robot forward.
     * The team's {@code DriveSubsystem} inverts all four for FTCLib field-centric,
     * but Pedro drives the raw motors itself, so the conventional mecanum setup
     * (left side reversed) is the right starting point. Verify by commanding
     * forward in a tuner and confirming the robot goes forward, not backward.
     */
    public static DcMotorSimple.Direction LEFT_FRONT_DIR = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction LEFT_REAR_DIR = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction RIGHT_FRONT_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction RIGHT_REAR_DIR = DcMotorSimple.Direction.REVERSE;

    /** Max forward velocity (in/s). GoBilda spec: 312 RPM × 12.862 in/rev ÷ 60 = 67 in/s. */
    public static double X_VELOCITY = 67.0;   // in/s  — GoBilda 5.57 ft/s theoretical
    /** Max strafe velocity (in/s). Typically ~75% of forward for mecanum. */
    public static double Y_VELOCITY = 50.0;   // in/s  — tune with Pedro Lateral Velocity Tuner
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
