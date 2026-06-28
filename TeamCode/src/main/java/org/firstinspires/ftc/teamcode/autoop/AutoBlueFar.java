package org.firstinspires.ftc.teamcode.autoop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.teleop.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.ShooterSubsystem;

/**
 * Blue-alliance far-side autonomous routine. (TEST — not competition-critical)
 *
 * Start = Shoot position: robot begins already at the shooting spot.
 *
 * Path layout (all heading 180°):
 *   step 0  SHOOTING  fire at START (68,6) — already at shoot pos
 *   step 1  PATHING   (68,6)→(39,35)→(8,35)→(68,6)   [intake, pre-spin on return at idx 2]
 *   step 2  SHOOTING  fire at (68,6)
 *   step 3  PATHING   (68,6)→(1,8)→(68,6)             [intake, pre-spin on return at idx 1]
 *   step 4  SHOOTING  fire at (68,6)
 */
@Config
@Autonomous(name = "Auto Blue Far", group = "Competition")
public class AutoBlueFar extends LinearOpMode {

    // ── Field poses (heading 180°) ────────────────────, reversed]─────────────────────────
    private static final double HEADING      = Math.toRadians(180);
    private static final Pose START_SHOOT    = new Pose(47.000,  10.000, HEADING);
    private static final Pose BALL1_MID      = new Pose(46.500, 37.000, HEADING);
    private static final Pose BALL1          = new Pose(18.000, 37.000, HEADING);
    private static final Pose BALL2          = new Pose(18.000,  11.000, HEADING);
    private static final Pose SHOOT          = new Pose(55.000, 10.000, HEADING);
    private static final Pose FINAL          = new Pose(45.000, 15.000, HEADING);

    // ── Shooter constants (tune from FTC Dashboard) ────────────────────────────
    public static double SHOOT_RPM            = 5100.0;
    public static double SHOOT_HOOD_POS       = 0.0;
    public static long   SHOOT_FIRE_MS        = 1500;
    public static double SHOOT_RPM_TOLERANCE  = 400.0;
    /** Safety-net wait after arriving — shooter is already at speed so this rarely triggers. */
    public static long   SHOOT_SPINUP_TIMEOUT_MS = 1500;
    /** Max time to wait for the turret to lock on the aim offset before firing anyway. */
    public static long   SHOOT_TURRET_LOCK_TIMEOUT_MS = 1000;

    // ── Intake ─────────────────────────────────────────────────────────────────
    public static double INTAKE_POWER = 1.0;

    // ── Ball 2 repeat count ────────────────────────────────────────────────────
    // Number of times to run the ball-2 pickup + shoot cycle. Set to 0 to skip entirely.
    public static int BALL2_LOOPS = 3;

    // ── Ball 2 shoot drift ─────────────────────────────────────────────────────
    // During each ball-2 shoot the robot slowly drifts this many inches to its left
    // (robot-relative: +Y field direction when heading is 180°) toward the wall.
    // Flip sign to drift right instead. Set to 0 to disable.
    public static double BALL2_DRIFT_INCHES = 11.8; // ~30 cm

    // ── Start delay ────────────────────────────────────────────────────────────
    public static long START_DELAY_MS = 1500;

    // ── State machine ──────────────────────────────────────────────────────────
    private enum FsmState { WAIT, PATHING, SHOOTING, DONE }

    private PedroAutoRunner  runner;
    private Follower         follower;
    private ShooterSubsystem shooter;
    private HoodSubsystem    hood;
    private IntakeSubsystem  intake;

    private FsmState state          = FsmState.PATHING;
    private int      step           = 0;
    private long     stateEnteredMs = 0;
    private int      ball2Done       = 0;     // how many complete ball-2 pick+shoot cycles have finished
    private boolean  ball2NeedsShoot = false; // true after a ball2 path, until the shoot completes
    private boolean  finalPath       = false; // true while following the end-of-auto FINAL path

    // WAIT sub-state
    private long    waitStartMs  = 0;

    // SHOOTING sub-state
    private boolean shooterFired = false;
    private long    fireStartMs  = 0;

    // Turret tracking — enabled only during SHOOTING and on the return leg of each ball path.
    private boolean turretTrackingEnabled = false;

    @Override
    public void runOpMode() {
        runner   = new PedroAutoRunner(hardwareMap);
        follower = runner.getFollower();
        shooter  = new ShooterSubsystem(hardwareMap);
        hood     = new HoodSubsystem(hardwareMap);
        intake   = new IntakeSubsystem(hardwareMap);

        runner.setStartPose(START_SHOOT);
        hood.setPosition(SHOOT_HOOD_POS);
        shooter.switchPipeline(ShooterConfig.APRILTAG_PIPELINE);

        telemetry.addLine("Auto Blue Far — waiting for start");
        telemetry.addData("SHOOT_RPM",      SHOOT_RPM);
        telemetry.addData("SHOOT_HOOD_POS", SHOOT_HOOD_POS);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Start flywheel immediately — robot is already at the shoot position.
        shooter.setShooterVelocityRpm(SHOOT_RPM);
        enterStep();

        while (opModeIsActive() && !isStopRequested()) {
            Pose currentPose = follower.getPose();
            shooter.cacheLimelightResult();
            shooter.setRobotPose(currentPose.getX(), currentPose.getY(),
                    Math.toDegrees(currentPose.getHeading()));

            switch (state) {
                case WAIT:     tickWait();     break;
                case PATHING:  tickPathing();  break;
                case SHOOTING: tickShooting(); break;
                case DONE:                     break;
            }

            follower.update();
            if (turretTrackingEnabled) {
                shooter.runTurretControl(0, false);
            } else {
                shooter.setTurretPower(0);
            }
            shooter.updatePID();
            renderTelemetry();
            telemetry.update();
        }

        intake.setPower(0);
        shooter.setShooterVelocityRpm(0);
        shooter.stopLimelight();
    }

    // ── Step sequencer ─────────────────────────────────────────────────────────

    private void enterStep() {
        stateEnteredMs = System.currentTimeMillis();
        switch (step) {
            case 0: enterWait();                      break;
            case 1: enterShooting(false);             break;
            case 2: enterPathing(chainBall1(), true); break;
            case 3: enterShooting(false);             break;
            default:
                if (ball2NeedsShoot) {
                    ball2NeedsShoot = false;
                    ball2Done++;
                    enterShooting(true);
                } else if (ball2Done < BALL2_LOOPS) {
                    ball2NeedsShoot = true;
                    enterPathing(chainBall2(), true);
                } else {
                    finalPath = true;
                    enterPathing(chainFinal(), false);
                }
                break;
        }
    }

    private void advance() { step++; enterStep(); }

    // ── WAIT ───────────────────────────────────────────────────────────────────

    private void enterWait() {
        state                 = FsmState.WAIT;
        waitStartMs           = System.currentTimeMillis();
        turretTrackingEnabled = false;
        intake.setPower(0);
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        shooter.setShooterVelocityRpm(SHOOT_RPM);
    }

    private void tickWait() {
        if (System.currentTimeMillis() - waitStartMs >= START_DELAY_MS) advance();
    }

    // ── PATHING ────────────────────────────────────────────────────────────────

    private void enterPathing(PathChain chain, boolean runIntake) {
        state                 = FsmState.PATHING;
        turretTrackingEnabled = false; // re-enabled in tickPathing when the return leg starts
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        intake.setPower(runIntake ? INTAKE_POWER : 0);
        hood.setPosition(SHOOT_HOOD_POS);
        shooter.setShooterVelocityRpm(SHOOT_RPM);
        runner.followPath(chain);
    }

    private void tickPathing() {
        // Enable turret tracking once the robot is on the return leg heading back to START_SHOOT.
        // chainBall1 (step 2): 3 segments — return is segment index 2.
        // chainBall2 (step 4): 2 segments — return is segment index 1.
        if (!turretTrackingEnabled) {
            int idx = follower.getChainIndex();
            // ball1 chain (step 2): 3 segments, return is idx 2.
            // ball2 chains (step >= 4): 2 segments, return is idx 1.
            if ((step == 2 && idx >= 2) || (step >= 4 && idx >= 1)) {
                turretTrackingEnabled = true;
            }
        }
        if (!runner.isBusy()) {
            if (finalPath) state = FsmState.DONE;
            else           advance();
        }
    }

    // ── SHOOTING ───────────────────────────────────────────────────────────────

    private void enterShooting(boolean driftLeft) {
        state                 = FsmState.SHOOTING;
        shooterFired          = false;
        fireStartMs           = 0;
        turretTrackingEnabled = true;
        intake.setPower(0);
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        shooter.setShooterVelocityRpm(SHOOT_RPM);
        hood.setPosition(SHOOT_HOOD_POS);
        if (driftLeft && BALL2_DRIFT_INCHES != 0) {
            // Start a slow lateral drift toward the wall while the turret locks and RPM spins up.
            // follower.update() runs every loop so Pedro executes this concurrently with tickShooting.
            // tickPathing is NOT called during SHOOTING, so path completion won't trigger advance().
            Pose cur = follower.getPose();
            Pose driftEnd = new Pose(cur.getX(), cur.getY() - BALL2_DRIFT_INCHES, HEADING);
            runner.followPath(follower.pathBuilder()
                    .addPath(new BezierLine(cur, driftEnd))
                    .setConstantHeadingInterpolation(HEADING)
                    .build());
        }
    }

    private void tickShooting() {
        long elapsed = System.currentTimeMillis() - stateEnteredMs;
        if (!shooterFired) {
            double targetRpm = shooter.getEffectiveTargetRpm();
            boolean atSpeed      = Math.abs(shooter.getShooterVelocityRpm() - targetRpm) < SHOOT_RPM_TOLERANCE;
            boolean timedOut     = elapsed > SHOOT_SPINUP_TIMEOUT_MS;
            // Only fire once the turret has settled at the aim offset — prevents firing while the
            // PID is still hunting, which causes the turret to oscillate side-to-side mid-shot.
            boolean turretLocked = shooter.isTurretLocked() || elapsed > SHOOT_TURRET_LOCK_TIMEOUT_MS;
            if ((atSpeed || timedOut) && turretLocked) {
                shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
                intake.setPower(INTAKE_POWER);
                shooterFired = true;
                fireStartMs  = System.currentTimeMillis();
            }
        } else if (System.currentTimeMillis() - fireStartMs >= SHOOT_FIRE_MS) {
            shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
            intake.setPower(0);
            advance();
        }
    }

    // ── Path chains ────────────────────────────────────────────────────────────

    /** (68,6) → (39,35) → (8,35) → (68,6) */
    private PathChain chainBall1() {
        return follower.pathBuilder()
                .addPath(new BezierLine(START_SHOOT, BALL1_MID))
                .setConstantHeadingInterpolation(HEADING)
                .addPath(new BezierLine(BALL1_MID, BALL1))
                .setConstantHeadingInterpolation(HEADING)
                .addPath(new BezierLine(BALL1, SHOOT))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    /** (68,6) → (1,8) → (68,6) */
    private PathChain chainBall2() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SHOOT, BALL2))
                .setConstantHeadingInterpolation(HEADING)
                .addPath(new BezierLine(BALL2, SHOOT))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    private PathChain chainFinal() {
        Pose cur = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(cur, FINAL))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    // ── Telemetry ──────────────────────────────────────────────────────────────

    private void renderTelemetry() {
        Pose p = follower.getPose();
        telemetry.addData("State",   "%s (step %d)", state, step);
        telemetry.addData("Pose",    "(%.1f, %.1f) %.0f°",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        Double llTx  = shooter.getTrackedTagTx();
        double estTx = shooter.getEstimatedTx();
        telemetry.addData("Turret",  "%.1f deg  LLtx=%s  estTx=%s  dist %.0f cm",
                shooter.getTurretAngleDeg(),
                llTx != null ? String.format("%.1f°", llTx) : "no tag",
                Double.isNaN(estTx) ? "?" : String.format("%.1f°", estTx),
                shooter.getTrackedTagDistanceCm());
        telemetry.addData("LL",      shooter.getLimelightDebugInfo());
        telemetry.addData("Hood",    "%.3f", hood.getPosition());
        telemetry.addData("RPM",     "%.0f / %.0f  fired=%b  turretLocked=%b",
                shooter.getShooterVelocityRpm(), SHOOT_RPM, shooterFired, shooter.isTurretLocked());
        if (state == FsmState.PATHING) {
            telemetry.addData("PathIdx", "%d", follower.getChainIndex());
        }
    }
}
