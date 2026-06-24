package org.firstinspires.ftc.teamcode.autoop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.teleop.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.ShooterSubsystem;

/**
 * FSM autonomous that drives the route defined by the original TestAuto path and layers
 * behaviour onto the named field poses:
 *
 *   - At the SHOOT pose (53, 88): stop, aim at the goal AprilTag, and fire (SHOOTING).
 *   - At each ball zone (42, 59) and (19, 59): look for a ball with the Limelight colour-blob
 *     pipeline (SEARCH_BALLS), grab it (INTAKE_BALL), then path back to the shoot pose and fire.
 *
 * Every loop, regardless of state, the turret auto-aims at the goal tag (via the shared
 * ShooterSubsystem auto-aim) and the flywheel PID is serviced. The flywheel is pre-spun on
 * the way back to the shoot pose so a shot does not pay a full cold spin-up, and the shot RPM
 * is taken from the distance polynomial whenever the goal tag is in view.
 *
 * States:
 *   PATHING      — Pedro follows a path segment to a waypoint, then advances.
 *   SHOOTING     — flywheel to the (distance-compensated) target RPM, open the stopper to
 *                  fire, close, advance.
 *   SEARCH_BALLS — STUB. Switches to the colour pipeline and (for now) proceeds immediately.
 *                  Real vision search goes in searchForBall() later.
 *   INTAKE_BALL  — STUB. Drives straight forward while running the intake for a fixed time.
 *                  A smarter ball-tracking version replaces this later.
 *   DONE         — routine finished.
 *
 * Step sequence:
 *   0  PATHING   START (21,119) -> SHOOT (53,88)   [pre-spin flywheel]
 *   1  SHOOTING  fire at (53,88)
 *   2  PATHING   SHOOT (53,88) -> BALL_1 (42,59)
 *   3  SEARCH_BALLS at (42,59)            [stub]
 *   4  INTAKE_BALL  drive forward + intake [stub]
 *   5  PATHING   current pose -> SHOOT (53,88)     [pre-spin flywheel]
 *   6  SHOOTING  fire at (53,88)
 *   7  PATHING   SHOOT (53,88) -> BALL_2 (19,59)
 *   8  SEARCH_BALLS at (19,59)            [stub]
 *   9  INTAKE_BALL  drive forward + intake [stub]
 *  10  PATHING   current pose -> SHOOT (53,88)     [pre-spin flywheel]
 *  11  SHOOTING  fire at (53,88)
 *  12  DONE
 *
 * All tuning values are @Config (live-editable from FTC Dashboard / Panels).
 */
@Config
@Autonomous(name = "Test Auto", group = "Pedro")
public class TestAuto extends LinearOpMode {

    // ── Field poses (Pedro coordinates: inches, heading radians) ─────────────
    // HEADING is both the robot's start orientation AND the heading every path holds.
    // Set it to match how the robot is PHYSICALLY placed at the start, or the localizer's
    // frame is rotated and every path drives off in the wrong direction.
    private static final double HEADING = Math.toRadians(90);
    private static final Pose START  = new Pose(21.000, 119.000, HEADING);
    private static final Pose SHOOT  = new Pose(53.000, 88.000, HEADING);
    private static final Pose BALL_1 = new Pose(42.000, 59.000, HEADING);
    private static final Pose BALL_2 = new Pose(19.000, 59.000, HEADING);

    // ── Vision / intake tuning ───────────────────────────────────────────────
    /** Limelight pipeline index configured for colour-blob (ball) detection. */
    public static int    COLOR_PIPELINE     = 1;
    /** Forward power while grabbing a ball. */
    public static double INTAKE_DRIVE_POWER = 0.30;
    /** Intake motor power while grabbing. */
    public static double INTAKE_POWER       = 1.0;
    /** How long INTAKE_BALL drives forward (ms) — the stub version is purely timed. */
    public static long   INTAKE_DRIVE_MS    = 1200;

    // ── Shooter tuning ───────────────────────────────────────────────────────
    /** RPM window around target before we consider the shooter ready to fire. */
    public static double SHOOT_RPM_TOLERANCE     = 200.0;
    /** Fire anyway after this many ms if RPM never converges. */
    public static long   SHOOT_SPINUP_TIMEOUT_MS = 3000;
    /** How long the stopper stays open while firing (ms). */
    public static long   SHOOT_FIRE_MS           = 600;

    // ── State machine ────────────────────────────────────────────────────────
    private enum FsmState { PATHING, SHOOTING, SEARCH_BALLS, INTAKE_BALL, DONE }

    private PedroAutoRunner  runner;
    private Follower         follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem  intake;

    private FsmState state          = FsmState.PATHING;
    private int      step           = 0;
    private long     stateEnteredMs = 0;

    // PATHING sub-state — pre-spin the flywheel while driving to the shoot pose.
    private boolean spinUpOnPath = false;

    // SHOOTING sub-state
    private boolean shooterFired  = false;
    private long    fireStartMs   = 0;
    private double  activeShootRpm = 0;

    @Override
    public void runOpMode() {
        runner   = new PedroAutoRunner(hardwareMap);
        follower = runner.getFollower();
        shooter  = new ShooterSubsystem(hardwareMap);
        intake   = new IntakeSubsystem(hardwareMap);

        runner.setStartPose(START);

        telemetry.addLine("Test Auto FSM — waiting for start");
        telemetry.addData("Start", "(%.1f, %.1f, %.0f deg)",
                START.getX(), START.getY(), Math.toDegrees(START.getHeading()));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        enterStep();

        while (opModeIsActive() && !isStopRequested()) {
            // Refresh Limelight + feed heading so the turret can auto-aim and counter-rotate
            // as the chassis moves. Done before the turret is driven below.
            shooter.cacheLimelightResult();
            shooter.setRobotHeading(Math.toDegrees(follower.getPose().getHeading()));

            switch (state) {
                case PATHING:      tickPathing();     break;
                case SHOOTING:     tickShooting();    break;
                case SEARCH_BALLS: tickSearchBalls(); break;
                case INTAKE_BALL:  tickIntakeBall();  break;
                case DONE:                            break;
            }

            follower.update();
            shooter.runTurretControl(0, false); // auto-aim turret at the goal tag every loop
            shooter.updatePID();                // service the flywheel PID every loop
            renderTelemetry();
            telemetry.update();
        }

        intake.setPower(0);
        shooter.setShooterVelocityRpm(0);
        shooter.stopLimelight();
    }

    // ── Step sequencer ────────────────────────────────────────────────────────

    private void enterStep() {
        stateEnteredMs = System.currentTimeMillis();
        switch (step) {
            case 0:  enterPathing(line(START, SHOOT),     true);  break; // -> shoot, pre-spin
            case 1:  enterShooting();                             break; // fire
            case 2:  enterPathing(line(SHOOT, BALL_1),    false); break; // -> ball zone 1
            case 3:  enterSearchBalls();                          break;
            case 4:  enterIntakeBall();                           break;
            case 5:  enterPathing(lineFromCurrent(SHOOT), true);  break; // back to shoot, pre-spin
            case 6:  enterShooting();                             break; // fire
            case 7:  enterPathing(line(SHOOT, BALL_2),    false); break; // -> ball zone 2
            case 8:  enterSearchBalls();                          break;
            case 9:  enterIntakeBall();                           break;
            case 10: enterPathing(lineFromCurrent(SHOOT), true);  break; // back to shoot, pre-spin
            case 11: enterShooting();                             break; // fire
            default: state = FsmState.DONE;                       break;
        }
    }

    private void advance() {
        step++;
        enterStep();
    }

    // ── PATHING ─────────────────────────────────────────────────────────────

    private void enterPathing(PathChain path, boolean spinUp) {
        state = FsmState.PATHING;
        spinUpOnPath = spinUp;
        intake.setPower(0);
        shooter.switchPipeline(ShooterConfig.APRILTAG_PIPELINE); // goal tag tracking
        if (!spinUp) shooter.setShooterVelocityRpm(0);
        runner.followPath(path);
    }

    private void tickPathing() {
        // Pre-spin the flywheel to the shot RPM while we drive to the shoot pose, so the
        // shot does not have to wait out a full cold spin-up on arrival.
        if (spinUpOnPath) shooter.setShooterVelocityRpm(shootTargetRpm());
        if (!runner.isBusy()) advance();
    }

    // ── SHOOTING (at the 53,88 shoot pose) ────────────────────────────────────

    private void enterShooting() {
        state         = FsmState.SHOOTING;
        shooterFired  = false;
        fireStartMs   = 0;
        activeShootRpm = shootTargetRpm();
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        shooter.setShooterVelocityRpm(activeShootRpm);
    }

    private void tickShooting() {
        long elapsed = System.currentTimeMillis() - stateEnteredMs;

        if (!shooterFired) {
            boolean atSpeed  = Math.abs(shooter.getShooterVelocityRpm() - activeShootRpm)
                    < SHOOT_RPM_TOLERANCE;
            boolean timedOut = elapsed > SHOOT_SPINUP_TIMEOUT_MS;

            if (atSpeed || timedOut) {
                shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
                shooterFired = true;
                fireStartMs  = System.currentTimeMillis();
            }
        } else if (System.currentTimeMillis() - fireStartMs >= SHOOT_FIRE_MS) {
            shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
            shooter.setShooterVelocityRpm(0);
            advance();
        }
    }

    /**
     * Target flywheel RPM for the current shot: the distance polynomial (with the
     * distance-ramped boost) when the goal tag is in view, otherwise the manual fallback.
     */
    private double shootTargetRpm() {
        double dist = shooter.getTrackedTagDistanceCm();
        if (dist > 0) {
            double d = Math.max(ShooterConfig.MIN_COMP_DISTANCE,
                    Math.min(dist, ShooterConfig.MAX_COMP_DISTANCE));
            return ShooterConfig.hoodTuneAngle(d) * ShooterConfig.distanceBoost(d);
        }
        return ShooterConfig.MANUAL_TARGET_RPM;
    }

    // ── SEARCH_BALLS (STUB) ───────────────────────────────────────────────────

    private void enterSearchBalls() {
        state = FsmState.SEARCH_BALLS;
        intake.setPower(0);
        shooter.setShooterVelocityRpm(0);
        // Arm the colour-blob pipeline so the real search has data when implemented.
        shooter.switchPipeline(COLOR_PIPELINE);
    }

    private void tickSearchBalls() {
        // STUB: the real version will scan/steer with the Limelight colour blobs and only
        // advance once a ball is located. For now, proceed straight through to INTAKE_BALL.
        if (searchForBall()) {
            advance();
        }
    }

    /**
     * STUB hook for the colour-blob ball search. Returns true once a ball is found and the
     * robot is lined up to intake it. Currently always true (no-op search) so the routine
     * flows through to INTAKE_BALL. Fill this in later.
     */
    private boolean searchForBall() {
        return true;
    }

    // ── INTAKE_BALL (STUB: drive forward + run intake for a fixed time) ────────

    private void enterIntakeBall() {
        state = FsmState.INTAKE_BALL;
        intake.setPower(INTAKE_POWER);
        // Reactive drive so we can command a straight forward push toward the ball.
        follower.startTeleopDrive();
    }

    private void tickIntakeBall() {
        if (System.currentTimeMillis() - stateEnteredMs >= INTAKE_DRIVE_MS) {
            follower.setTeleOpDrive(0, 0, 0, true);
            intake.setPower(0);
            follower.breakFollowing();
            advance();
            return;
        }
        // Drive straight forward while the intake runs.
        follower.setTeleOpDrive(INTAKE_DRIVE_POWER, 0, 0, true);
    }

    // ── Path helpers ──────────────────────────────────────────────────────────

    /** Straight constant-heading line between two fixed poses. */
    private PathChain line(Pose from, Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    /** Straight constant-heading line from the robot's CURRENT pose to a fixed target.
     *  Used after INTAKE_BALL, where the robot has driven forward an unknown distance. */
    private PathChain lineFromCurrent(Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), to))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    private void renderTelemetry() {
        Pose p = follower.getPose();
        telemetry.addData("State", "%s (step %d)", state, step);
        telemetry.addData("Pose", "(%.1f, %.1f) %.0f deg",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("Following", runner.isBusy());
        telemetry.addData("Turret", "%.1f deg  tag dist %.0f cm",
                shooter.getTurretAngleDeg(), shooter.getTrackedTagDistanceCm());

        if (state == FsmState.SHOOTING || (state == FsmState.PATHING && spinUpOnPath)) {
            telemetry.addData("Shooter RPM", "%.0f / %.0f",
                    shooter.getShooterVelocityRpm(),
                    state == FsmState.SHOOTING ? activeShootRpm : shootTargetRpm());
            telemetry.addData("Fired", shooterFired);
        }
        if (state == FsmState.INTAKE_BALL) {
            telemetry.addData("Intake", "%d / %d ms",
                    System.currentTimeMillis() - stateEnteredMs, INTAKE_DRIVE_MS);
        }
    }
}
