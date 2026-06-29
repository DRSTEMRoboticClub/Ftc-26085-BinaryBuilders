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
 * Red-alliance far-side autonomous routine — mirror of AutoBlueFar.
 *
 * Mirror rule: X_red = 141.5 - X_blue, Y unchanged, heading = π - θ_blue (180°→0°).
 * Tracks AprilTag ID 24 (Red alliance hub).
 */
@Config
@Autonomous(name = "Auto Red Far", group = "Competition")
public class AutoRedFar extends LinearOpMode {

    // ── Field poses (heading 0°: robot faces +X) ──────────────────────────────
    private static final double HEADING   = Math.toRadians(0);
    private static final Pose START_SHOOT = new Pose( 94.500, 10.000, HEADING);
    private static final Pose BALL1_MID   = new Pose( 95.000, 33.000, HEADING);
    private static final Pose BALL1       = new Pose(126.500, 33.000, HEADING);
    private static final Pose BALL2       = new Pose(130.500,  6.000, HEADING);
    private static final Pose SHOOT       = new Pose( 86.500, 10.000, HEADING);
    private static final Pose FINAL       = new Pose( 96.500, 15.000, HEADING);
    private static final Pose GOAL        = new Pose(11.000, 135.000, 0);

    // ── Shooter constants (tune from FTC Dashboard) ────────────────────────────
    public static double SHOOT_RPM                   = 5000.0;
    public static double SHOOT_HOOD_POS              = 0;
    public static long   SHOOT_FIRE_MS               = 1500;
    public static double SHOOT_RPM_TOLERANCE         = 400.0;
    public static long   SHOOT_SPINUP_TIMEOUT_MS     = 500;
    public static long   SHOOT_TURRET_LOCK_TIMEOUT_MS = 1000;

    // ── Intake ─────────────────────────────────────────────────────────────────
    public static double INTAKE_POWER              = 1.0;
    public static double BALL2_RETURN_INTAKE_POWER = 1.0;

    // ── Ball 2 repeat count ────────────────────────────────────────────────────
    public static int BALL2_LOOPS = 2;

    // ── Ball 2 shoot drift ─────────────────────────────────────────────────────
    public static long   BALL2_DRIFT_MS    = 600;
    public static double BALL2_DRIFT_POWER = -1.0;

    // ── Heading correction during shooting ────────────────────────────────────
    public static double HEADING_CORRECTION_P = 1.5;

    // ── Ball 2 dwell at pickup point ──────────────────────────────────────────
    public static long BALL2_WAIT_MS = 500;

    // ── Start delay ────────────────────────────────────────────────────────────
    public static long START_DELAY_MS = 1500;

    // ── State machine ──────────────────────────────────────────────────────────
    private enum FsmState { WAIT, PATHING, INTAKE_WAIT, SHOOTING, DONE }

    private PedroAutoRunner  runner;
    private Follower         follower;
    private ShooterSubsystem shooter;
    private HoodSubsystem    hood;
    private IntakeSubsystem  intake;

    private FsmState state           = FsmState.PATHING;
    private int      step            = 0;
    private long     stateEnteredMs  = 0;
    private int      ball2Done       = 0;
    private boolean  ball2NeedsWait   = false;
    private boolean  ball2NeedsReturn = false;
    private boolean  ball2NeedsShoot  = false;
    private boolean  finalPath        = false;
    private boolean  polynomialActive = false;

    private long    waitStartMs     = 0;
    private long    intakeWaitStart = 0;

    private boolean shooterFired    = false;
    private long    fireStartMs     = 0;
    private boolean driftInProgress = false;
    private long    driftStartMs    = 0;

    private boolean turretTrackingEnabled = false;

    @Override
    public void runOpMode() {
        runner   = new PedroAutoRunner(hardwareMap);
        follower = runner.getFollower();
        shooter  = new ShooterSubsystem(hardwareMap);
        hood     = new HoodSubsystem(hardwareMap);
        intake   = new IntakeSubsystem(hardwareMap);

        ShooterConfig.TRACKED_TAG_ID = 24; // Red alliance hub tag

        runner.setStartPose(START_SHOOT);
        hood.setPosition(SHOOT_HOOD_POS);
        shooter.switchPipeline(ShooterConfig.APRILTAG_PIPELINE);

        telemetry.addLine("Auto Red Far — waiting for start");
        telemetry.addData("SHOOT_RPM",      SHOOT_RPM);
        telemetry.addData("SHOOT_HOOD_POS", SHOOT_HOOD_POS);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        shooter.setShooterVelocityRpm(SHOOT_RPM);
        enterStep();

        while (opModeIsActive() && !isStopRequested()) {
            Pose currentPose = follower.getPose();
            shooter.cacheLimelightResult();
            shooter.setRobotPose(currentPose.getX(), currentPose.getY(),
                    Math.toDegrees(currentPose.getHeading()));

            switch (state) {
                case WAIT:        tickWait();        break;
                case PATHING:     tickPathing();     break;
                case INTAKE_WAIT: tickIntakeWait();  break;
                case SHOOTING:    tickShooting();    break;
                case DONE:                           break;
            }

            if (polynomialActive && ShooterConfig.USE_DISTANCE_COMPENSATION) {
                double dist = shooter.getTrackedTagDistanceCm();
                if (dist > 0) {
                    double d = Math.max(ShooterConfig.MIN_COMP_DISTANCE,
                                        Math.min(dist, ShooterConfig.MAX_COMP_DISTANCE));
                    shooter.setAutoShootRpmOverride(ShooterConfig.hoodTuneAngle(d));
                    hood.setPosition(ShooterConfig.hoodPitch(d));
                } else {
                    shooter.clearAutoShootRpmOverride();
                }
            } else {
                shooter.clearAutoShootRpmOverride();
            }

            follower.update();
            if (turretTrackingEnabled) {
                if (shooter.getTrackedTagTx() != null) {
                    shooter.runTurretControl(0, false);
                } else {
                    shooter.holdTurretAtAngle(computeGoalTurretAngleDeg(GOAL));
                }
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
            case 0: enterWait();                                                    break;
            case 1: enterShooting(false, false);                                    break;
            case 2: polynomialActive = true; enterPathing(chainBall1(), true);      break;
            case 3: enterShooting(true, true);                                      break;
            default:
                if (ball2NeedsShoot) {
                    ball2NeedsShoot = false;
                    ball2Done++;
                    enterShooting(true, true);
                } else if (ball2NeedsReturn) {
                    ball2NeedsReturn = false;
                    ball2NeedsShoot  = true;
                    enterPathing(chainBall2Return(), true);
                } else if (ball2NeedsWait) {
                    ball2NeedsWait   = false;
                    ball2NeedsReturn = true;
                    enterIntakeWait();
                } else if (ball2Done < BALL2_LOOPS) {
                    ball2NeedsWait = true;
                    enterPathing(chainBall2Out(), true);
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
    }

    private void tickIntakeWait() {
        shooter.setShooterVelocityRpm(SHOOT_RPM);
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
        if (step >= 4 && follower.getChainIndex() >= 1) {
            intake.setPower(BALL2_RETURN_INTAKE_POWER);
        }
        if (!runner.isBusy()) {
            if (finalPath) state = FsmState.DONE;
            else           advance();
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
        double turnCorrection = Math.max(-0.5, Math.min(0.5, headingError * HEADING_CORRECTION_P));

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
            double targetRpm     = shooter.getEffectiveTargetRpm();
            boolean atSpeed      = Math.abs(shooter.getShooterVelocityRpm() - targetRpm) < SHOOT_RPM_TOLERANCE;
            boolean timedOut     = elapsed > SHOOT_SPINUP_TIMEOUT_MS;
            boolean hasTag       = shooter.getTrackedTagTx() != null;
            boolean turretReady  = hasTag && (shooter.isTurretLocked() || elapsed > SHOOT_TURRET_LOCK_TIMEOUT_MS);
            if ((atSpeed || timedOut) && turretReady && !driftInProgress) {
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

    private PathChain chainBall2Out() {
        Pose cur = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(cur, BALL2))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    private PathChain chainBall2Return() {
        return follower.pathBuilder()
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

    private double computeGoalTurretAngleDeg(Pose goalPose) {
        Pose cur = follower.getPose();
        double dx = goalPose.getX() - cur.getX();
        double dy = goalPose.getY() - cur.getY();
        double goalHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(cur.getHeading());
        return normalizeDeg(goalHeadingDeg - robotHeadingDeg);
    }

    private static double normalizeDeg(double deg) {
        while (deg >  180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    // ── Telemetry ──────────────────────────────────────────────────────────────

    private void renderTelemetry() {
        Pose p = follower.getPose();
        telemetry.addData("State",   "%s (step %d)", state, step);
        telemetry.addData("Pose",    "(%.1f, %.1f) %.0f°",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        Double tgtTx = shooter.getTrackedTagTx();
        Double rawTx = shooter.getRawTagTx();
        double estTx = shooter.getEstimatedTx();
        Double fallbackGoal = tgtTx == null ? computeGoalTurretAngleDeg(GOAL) : null;
        telemetry.addData("Turret",  "%.1f deg  tgtTx=%s rawTx=%s  estTx=%s  dist %.0f cm",
                shooter.getTurretAngleDeg(),
                tgtTx != null ? String.format("%.1f°", tgtTx) : "no tag",
                rawTx != null ? String.format("%.1f°", rawTx) : "-",
                Double.isNaN(estTx) ? "?" : String.format("%.1f°", estTx),
                shooter.getTrackedTagDistanceCm());
        telemetry.addData("GoalAim", tgtTx != null ? "tag" : String.format("%.1f°", fallbackGoal));
        telemetry.addData("LL",      shooter.getLimelightDebugInfo());
        telemetry.addData("Hood",    "%.3f", hood.getPosition());
        telemetry.addData("RPM",     "%.0f / %.0f  fired=%b  turretLocked=%b",
                shooter.getShooterVelocityRpm(), SHOOT_RPM, shooterFired, shooter.isTurretLocked());
        if (state == FsmState.PATHING) {
            telemetry.addData("PathIdx", "%d", follower.getChainIndex());
        }
    }
}
