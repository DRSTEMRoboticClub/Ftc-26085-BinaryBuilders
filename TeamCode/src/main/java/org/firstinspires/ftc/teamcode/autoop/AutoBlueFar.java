package org.firstinspires.ftc.teamcode.autoop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
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
    private static final Pose BALL1_MID      = new Pose(46.500, 33.000, HEADING);
    private static final Pose BALL1          = new Pose(15.000, 33.000, HEADING);
    private static final Pose SHOOT          = new Pose(55.000, 10.000, HEADING);
    private static final Pose FINAL          = new Pose(45.000, 15.000, HEADING);
    private static final Pose GOAL           = new Pose(130.500, 135.000, 0);

    // ── Shooter constants (tune from FTC Dashboard) ────────────────────────────
    public static double SHOOT_RPM            = 5000.0;
    public static double SHOOT_HOOD_POS       = 0.12;
    public static long   SHOOT_FIRE_MS        = 1500;
    public static double SHOOT_RPM_TOLERANCE  = 400.0;
    /** Safety-net wait after arriving — shooter is already at speed so this rarely triggers. */
    public static long   SHOOT_SPINUP_TIMEOUT_MS = 3500;
    /** Max time to wait for the turret to lock on the aim offset before firing anyway. */
    public static long   SHOOT_TURRET_LOCK_TIMEOUT_MS = 1000;

    // ── Intake ─────────────────────────────────────────────────────────────────
    public static double INTAKE_POWER = 1.0;
    /** Intake power on the return leg from BALL2 to the shoot position. */
    public static double BALL2_RETURN_INTAKE_POWER = 1.0;

    // ── Ball 2 repeat count ────────────────────────────────────────────────────
    // Number of times to run the ball-2 pickup + shoot cycle. Set to 0 to skip entirely.
    public static int BALL2_LOOPS = 2;

    // ── Ball 2 shoot drift ─────────────────────────────────────────────────────
    // Robot drifts at full strafe power for this many milliseconds before firing.
    // Set to 0 to disable. Flip BALL2_DRIFT_POWER sign to reverse direction.
    public static long   BALL2_DRIFT_MS    = 300;
    public static double BALL2_DRIFT_POWER = -1.0; // -1 = left in robot frame at heading 180°

    // ── Heading PID ───────────────────────────────────────────────────────────
    public static double HEADING_KP            = 1.5;
    public static double HEADING_KI            = 0.0;
    public static double HEADING_KD            = 0.05;
    public static double HEADING_TOLERANCE_DEG = 3.0;

    // ── Ball 2 timed drive ─────────────────────────────────────────────────────
    private static final long   BALL2_OUT_MS     = 1300;  // time driving toward ball 2
    private static final long   BALL2_RETURN_MS  = 900;  // time driving back to shoot pos
    private static final double BALL2_OUT_FWD    = 0.8;   // forward power (robot frame) toward ball 2
    private static final double BALL2_OUT_STRAFE = 0.0;   // strafe power (robot frame) toward ball 2

    // ── Ball 2 dwell at pickup point ──────────────────────────────────────────
    public static long BALL2_WAIT_MS = 500;

    // ── Start delay ────────────────────────────────────────────────────────────
    public static long START_DELAY_MS = 1500;

    // ── State machine ──────────────────────────────────────────────────────────
    private enum FsmState { WAIT, PATHING, INTAKE_WAIT, TIMED_DRIVE, SHOOTING, DONE }

    private PedroAutoRunner  runner;
    private Follower         follower;
    private ShooterSubsystem shooter;
    private HoodSubsystem    hood;
    private IntakeSubsystem  intake;

    private FsmState state          = FsmState.PATHING;
    private int      step           = 0;
    private long     stateEnteredMs = 0;
    private int      ball2Done       = 0;
    private boolean  ball2NeedsWait   = false; // arrived at BALL2, waiting before returning
    private boolean  ball2NeedsReturn = false; // wait done, follow return path
    private boolean  ball2NeedsShoot  = false; // return path done, time to shoot
    private boolean  finalPath        = false;
    private boolean  polynomialActive = false;

    // WAIT / INTAKE_WAIT sub-state
    private long    waitStartMs     = 0;
    private long    intakeWaitStart = 0;

    // TIMED_DRIVE sub-state
    private long   timedDriveEndMs  = 0;
    private double timedDriveFwd    = 0;
    private double timedDriveStrafe = 0;

    // SHOOTING sub-state
    private boolean shooterFired    = false;
    private long    fireStartMs     = 0;
    private boolean driftInProgress = false; // true while the timed drift strafe is active
    private long    driftStartMs    = 0;

    // Turret tracking — enabled only during SHOOTING and on the return leg of each ball path.
    private boolean turretTrackingEnabled = false;

    private PIDController headingPid;

    @Override
    public void runOpMode() {
        runner   = new PedroAutoRunner(hardwareMap);
        follower = runner.getFollower();
        shooter  = new ShooterSubsystem(hardwareMap);
        hood     = new HoodSubsystem(hardwareMap);
        intake   = new IntakeSubsystem(hardwareMap);

        headingPid = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);

        runner.setStartPose(START_SHOOT);
        hood.setPosition(SHOOT_HOOD_POS);

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
            shooter.setRobotPose(currentPose.getX(), currentPose.getY(),
                    Math.toDegrees(currentPose.getHeading()));

            switch (state) {
                case WAIT:        tickWait();        break;
                case PATHING:     tickPathing();     break;
                case INTAKE_WAIT: tickIntakeWait();  break;
                case TIMED_DRIVE: tickTimedDrive();  break;
                case SHOOTING:    tickShooting();    break;
                case DONE:                           break;
            }

            // Polynomial RPM + hood — active during pathing from step 2 onward.
            // Never applied during SHOOTING: the stopper fires at hardcoded SHOOT_RPM always.
            if (polynomialActive && ShooterConfig.USE_DISTANCE_COMPENSATION
                    && state != FsmState.SHOOTING) {
                double dist = goalDistanceCm();
                if (dist > 0) {
                    double d = Math.max(ShooterConfig.MIN_COMP_DISTANCE,
                                        Math.min(dist, ShooterConfig.MAX_COMP_DISTANCE));
                    shooter.setAutoShootRpmOverride(ShooterConfig.hoodTuneAngle(d));
                    hood.setPosition(ShooterConfig.hoodPitch(d));
                } else {
                    shooter.clearAutoShootRpmOverride(); // fallback to SHOOT_RPM set in enterShooting/enterPathing
                }
            } else {
                shooter.clearAutoShootRpmOverride();
            }

            follower.update();
            if (turretTrackingEnabled) {
                shooter.holdTurretAtAngle(computeGoalTurretAngleDeg(GOAL));
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
            case 1: enterShooting(false, false);                    break;
            case 2: polynomialActive = true; enterPathing(chainBall1(), true); break;
            case 3: enterShooting(true,  true);       break;
            default:
                if (ball2NeedsShoot) {
                    ball2NeedsShoot = false;
                    ball2Done++;
                    enterShooting(true, true);
                } else if (ball2NeedsReturn) {
                    ball2NeedsReturn = false;
                    ball2NeedsShoot  = true;
                    enterTimedDrive(-BALL2_OUT_FWD, -BALL2_OUT_STRAFE, BALL2_RETURN_MS, true, false);
                } else if (ball2NeedsWait) {
                    ball2NeedsWait   = false;
                    ball2NeedsReturn = true;
                    enterIntakeWait();
                } else if (ball2Done < BALL2_LOOPS) {
                    ball2NeedsWait = true;
                    enterTimedDrive(BALL2_OUT_FWD, BALL2_OUT_STRAFE, BALL2_OUT_MS, true, false);
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

    // ── INTAKE_WAIT ────────────────────────────────────────────────────────────

    private void enterIntakeWait() {
        state           = FsmState.INTAKE_WAIT;
        intakeWaitStart = System.currentTimeMillis();
        intake.setPower(INTAKE_POWER);
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        shooter.setShooterVelocityRpm(SHOOT_RPM);
        headingPid.reset();
        follower.startTeleOpDrive();
    }

    private void tickIntakeWait() {
        shooter.setShooterVelocityRpm(SHOOT_RPM);
        follower.setTeleOpDrive(0, 0, computeHeadingCorrection(), true);
        if (System.currentTimeMillis() - intakeWaitStart >= BALL2_WAIT_MS) advance();
    }

    // ── PATHING ────────────────────────────────────────────────────────────────

    private void enterPathing(PathChain chain, boolean runIntake) {
        state                 = FsmState.PATHING;
        turretTrackingEnabled = false;
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        intake.setPower(runIntake ? INTAKE_POWER : 0);
        hood.setPosition(SHOOT_HOOD_POS);
        shooter.setShooterVelocityRpm(SHOOT_RPM);
        runner.followPath(chain);
    }

    private void tickPathing() {
        // Enable tracking only on the return-to-shoot leg of the ball1 chain (BALL1→SHOOT, index 2).
        if (step == 2 && follower.getChainIndex() >= 2) {
            turretTrackingEnabled = true;
        }
        if (!runner.isBusy()) {
            if (finalPath) state = FsmState.DONE;
            else           advance();
        }
    }

    // ── TIMED_DRIVE ────────────────────────────────────────────────────────────

    private void enterTimedDrive(double fwd, double strafe, long durationMs, boolean runIntake, boolean tracking) {
        state                 = FsmState.TIMED_DRIVE;
        turretTrackingEnabled = tracking;
        timedDriveFwd         = fwd;
        timedDriveStrafe      = strafe;
        timedDriveEndMs       = System.currentTimeMillis() + durationMs;
        intake.setPower(runIntake ? INTAKE_POWER : 0);
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        shooter.setShooterVelocityRpm(SHOOT_RPM);
        headingPid.reset();
        follower.startTeleOpDrive();
    }

    private void tickTimedDrive() {
        long remaining = timedDriveEndMs - System.currentTimeMillis();
        double scale = (timedDriveFwd > 0 && remaining < 150) ? 0.6 : 1.0;
        follower.setTeleOpDrive(timedDriveFwd * scale, timedDriveStrafe * scale, computeHeadingCorrection(), true);
        if (remaining <= 0) {
            follower.setTeleOpDrive(0, 0, 0, true);
            advance();
        }
    }

    // ── SHOOTING ───────────────────────────────────────────────────────────────

    private void enterShooting(boolean driftLeft, boolean autoAim) {
        state                 = FsmState.SHOOTING;
        shooterFired          = false;
        fireStartMs           = 0;
        turretTrackingEnabled = autoAim;
        intake.setPower(0);
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        shooter.setShooterVelocityRpm(SHOOT_RPM);
        hood.setPosition(SHOOT_HOOD_POS);
        follower.startTeleOpDrive();
        headingPid.reset();
        driftInProgress = false;
        driftStartMs    = 0;
        if (driftLeft && BALL2_DRIFT_MS > 0) {
            driftStartMs    = System.currentTimeMillis();
            driftInProgress = true;
        }
    }

    private void tickShooting() {
        long elapsed = System.currentTimeMillis() - stateEnteredMs;
        double headingError = HEADING - follower.getPose().getHeading();
        while (headingError >  Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        double turnCorrection = computeHeadingCorrection();

        double strafe = 0;
        if (driftInProgress) {
            if (System.currentTimeMillis() - driftStartMs < BALL2_DRIFT_MS) {
                strafe = BALL2_DRIFT_POWER;
            } else {
                driftInProgress = false;
            }
        }
        follower.setTeleOpDrive(0, strafe, turnCorrection, true);
        if (!shooterFired) {
            double targetRpm = shooter.getEffectiveTargetRpm();
            boolean atSpeed      = Math.abs(shooter.getShooterVelocityRpm() - targetRpm) < SHOOT_RPM_TOLERANCE;
            boolean timedOut     = elapsed > SHOOT_SPINUP_TIMEOUT_MS;
            // Only fire once the turret has settled at the aim offset — prevents firing while the
            // PID is still hunting, which causes the turret to oscillate side-to-side mid-shot.
            boolean turretReady  = shooter.isTurretLocked() || elapsed > SHOOT_TURRET_LOCK_TIMEOUT_MS;
            // Gate on heading settled — prevents firing while the chassis is still spinning from
            // drift. Falls back to firing after SHOOT_TURRET_LOCK_TIMEOUT_MS regardless.
            boolean headingOk    = Math.abs(headingError) < Math.toRadians(HEADING_TOLERANCE_DEG)
                    || elapsed > SHOOT_TURRET_LOCK_TIMEOUT_MS;
            if (timedOut || (atSpeed && turretReady && headingOk && !driftInProgress)) {
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

    private PathChain chainFinal() {
        Pose cur = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(cur, FINAL))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    private double computeGoalTurretAngleDeg(Pose goalPose) {
        Pose cur = follower.getPose();
        double dx = goalPose.getX() - cur.getX();
        double dy = goalPose.getY() - cur.getY();
        double goalHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(cur.getHeading());
        return normalizeDeg(goalHeadingDeg - robotHeadingDeg);
    }

    private double goalDistanceCm() {
        Pose cur = follower.getPose();
        double dx = GOAL.getX() - cur.getX();
        double dy = GOAL.getY() - cur.getY();
        return Math.sqrt(dx * dx + dy * dy) * 2.54;
    }

    private static double normalizeDeg(double deg) {
        while (deg >  180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    // ── Heading PID ────────────────────────────────────────────────────────────

    private double computeHeadingCorrection() {
        double error = HEADING - follower.getPose().getHeading();
        while (error >  Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return Math.max(-0.5, Math.min(0.5, headingPid.calculate(0, error)));
    }

    // ── Telemetry ──────────────────────────────────────────────────────────────

    private void renderTelemetry() {
        Pose p = follower.getPose();
        telemetry.addData("State",  "%s (step %d)", state, step);
        telemetry.addData("Pose",   "(%.1f, %.1f) %.0f°",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        double hErr = HEADING - p.getHeading();
        while (hErr >  Math.PI) hErr -= 2 * Math.PI;
        while (hErr < -Math.PI) hErr += 2 * Math.PI;
        telemetry.addData("Heading", "%.1f° (err %.1f°, tol %.1f°)",
                Math.toDegrees(p.getHeading()), Math.toDegrees(hErr), HEADING_TOLERANCE_DEG);
        telemetry.addData("Turret", "%.1f deg  goal=%.1f°  dist=%.0f cm  locked=%b",
                shooter.getTurretAngleDeg(),
                computeGoalTurretAngleDeg(GOAL),
                goalDistanceCm(),
                shooter.isTurretLocked());
        telemetry.addData("Hood",   "%.3f", hood.getPosition());
        telemetry.addData("RPM",    "%.0f / %.0f  fired=%b",
                shooter.getShooterVelocityRpm(), SHOOT_RPM, shooterFired);
        if (state == FsmState.PATHING) {
            telemetry.addData("PathIdx", "%d", follower.getChainIndex());
        }
    }
}
