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
 * Red-alliance far-side autonomous — geometry-only turret aiming (no Limelight).
 * Turret angle and distance compensation derived entirely from Pedro odometry pose.
 */
@Config
@Autonomous(name = "Auto Red Far Geo", group = "Competition")
public class AutoRedFarGeo extends LinearOpMode {

    // ── Field poses (heading 0°: robot faces +X) ──────────────────────────────
    private static final double HEADING   = Math.toRadians(0);
    private static final Pose START_SHOOT = new Pose( 94.500, 10.000, HEADING);
    private static final Pose BALL1_MID   = new Pose( 95.000, 33.000, HEADING);
    private static final Pose BALL1       = new Pose(126.500, 33.000, HEADING);
    private static final Pose SHOOT       = new Pose( 86.500, 10.000, HEADING);
    private static final Pose FINAL       = new Pose( 96.500, 15.000, HEADING);
    private static final Pose GOAL        = new Pose(11.000, 135.000, 0);

    // ── Shooter constants (tune from FTC Dashboard) ────────────────────────────
    public static double SHOOT_RPM                   = 5000.0;
    public static double SHOOT_HOOD_POS              = 0.12;
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
    public static long   BALL2_DRIFT_MS    = 300;
    public static double BALL2_DRIFT_POWER = -1.0;

    // ── Heading PID ───────────────────────────────────────────────────────────
    public static double HEADING_KP = 1.5;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.05;

    // ── Ball 2 timed drive ─────────────────────────────────────────────────────
    private static final long   BALL2_OUT_MS     = 1300;
    private static final long   BALL2_RETURN_MS  = 900;
    private static final double BALL2_OUT_FWD    = 0.8;
    private static final double BALL2_OUT_STRAFE = 0.0;

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

    // TIMED_DRIVE sub-state
    private long   timedDriveEndMs  = 0;
    private double timedDriveFwd    = 0;
    private double timedDriveStrafe = 0;

    private boolean shooterFired    = false;
    private long    fireStartMs     = 0;
    private boolean driftInProgress = false;
    private long    driftStartMs    = 0;

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

        telemetry.addLine("Auto Red Far Geo — waiting for start");
        telemetry.addData("SHOOT_RPM",      SHOOT_RPM);
        telemetry.addData("SHOOT_HOOD_POS", SHOOT_HOOD_POS);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

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

            if (polynomialActive && ShooterConfig.USE_DISTANCE_COMPENSATION) {
                double dist = goalDistanceCm();
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
            case 0: enterWait();                                               break;
            case 1: enterShooting(false, true);                                break;
            case 2: polynomialActive = true; enterPathing(chainBall1(), true); break;
            case 3: enterShooting(true, true);                                 break;
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
            double targetRpm    = shooter.getEffectiveTargetRpm();
            boolean atSpeed     = Math.abs(shooter.getShooterVelocityRpm() - targetRpm) < SHOOT_RPM_TOLERANCE;
            boolean timedOut    = elapsed > SHOOT_SPINUP_TIMEOUT_MS;
            boolean turretReady = shooter.isTurretLocked() || elapsed > SHOOT_TURRET_LOCK_TIMEOUT_MS;
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
