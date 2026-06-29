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
 * Red-alliance near-side autonomous routine — mirror of AutoBlueNear.
 *
 * Mirror rule: X_red = 141.5 - X_blue, Y unchanged, heading = π - θ_blue (180°→0°, 165°→15°).
 */
@Config
@Autonomous(name = "Auto Red Near", group = "Competition")
public class AutoRedNear extends LinearOpMode {

    // ── Field poses (heading 0°: robot faces +X) ──────────────────────────────
    private static final double HEADING    = Math.toRadians(0);
    private static final Pose START        = new Pose(120.500, 119.000, HEADING);
    private static final Pose SHOOT_START  = new Pose( 88.500,  88.000, HEADING);
    private static final Pose BALL1_SWEEP  = new Pose(102.500,  77.000, HEADING);
    private static final Pose BALL1        = new Pose(115.500,  77.000, HEADING);
    private static final Pose BALL2_SWEEP  = new Pose(98.500,  45.000, HEADING);
    private static final Pose BALL2        = new Pose(125.500,  45.000, HEADING);
    private static final Pose FINAL        = new Pose(131.500,  88.000, HEADING);
    private static final Pose SHOOT        = new Pose( 91.500,  83.000, HEADING);
    // RELEASE heading mirrors 165° → 15° (π - 165° = 15°)
    private static final Pose RELEASE      = new Pose(130.000,  61.500, Math.toRadians(15));

    public static int RELEASE_LOOPS = 0;

    // ── Shooter constants (tune from FTC Dashboard) ────────────────────────────
    public static double SHOOT_RPM            = 4100.0;
    public static double SHOOT_HOOD_POS       = 0.12;
    public static long   SHOOT_FIRE_MS        = 1500;
    public static double SHOOT_RPM_TOLERANCE  = 400.0;
    public static long   SHOOT_SPINUP_TIMEOUT_MS      = 0;
    public static long   SHOOT_TURRET_LOCK_TIMEOUT_MS = 1000;

    // ── Intake ─────────────────────────────────────────────────────────────────
    public static double INTAKE_POWER   = 1.0;
    public static long   INTAKE_WAIT_MS = 0;

    // ── State machine ──────────────────────────────────────────────────────────
    private enum FsmState { PATHING, SHOOTING, INTAKE_WAIT, DONE }

    private PedroAutoRunner  runner;
    private Follower         follower;
    private ShooterSubsystem shooter;
    private HoodSubsystem    hood;
    private IntakeSubsystem  intake;

    private FsmState state          = FsmState.PATHING;
    private int      step           = 0;
    private long     stateEnteredMs = 0;
    private boolean  finalPath      = false;

    private boolean turretTrackingEnabled = false;

    private boolean shooterFired = false;
    private long    fireStartMs  = 0;

    private long intakeWaitStart = 0;

    @Override
    public void runOpMode() {
        runner   = new PedroAutoRunner(hardwareMap);
        follower = runner.getFollower();
        shooter  = new ShooterSubsystem(hardwareMap);
        hood     = new HoodSubsystem(hardwareMap);
        intake   = new IntakeSubsystem(hardwareMap);

        ShooterConfig.TRACKED_TAG_ID = 24; // Red alliance hub tag
        runner.setStartPose(START);
        hood.setPosition(SHOOT_HOOD_POS);
        shooter.switchPipeline(ShooterConfig.APRILTAG_PIPELINE);

        telemetry.addLine("Auto Red Near — waiting for start");
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
                case PATHING:     tickPathing();     break;
                case SHOOTING:    tickShooting();    break;
                case INTAKE_WAIT: tickIntakeWait();  break;
                case DONE:                           break;
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
            case 0: enterPathing(chainApproach(), false); break;
            case 1: enterShooting();                      break;
            case 2: enterPathing(chainBall1(),  true);    break;
            case 3: enterShooting();                      break;
            case 4: enterPathing(chainBall2(),  true);    break;
            case 5: enterShooting();                      break;
            default:
                int relativeStep = step - 6;
                int cycleNum  = relativeStep / 4;
                int cycleStep = relativeStep % 4;
                if (cycleNum >= RELEASE_LOOPS) {
                    finalPath = true;
                    enterPathing(chainFinal(), false);
                } else {
                    switch (cycleStep) {
                        case 0: enterPathing(chainShootToRelease(), true); break;
                        case 1: enterIntakeWait();                         break;
                        case 2: enterPathing(chainReleaseToShoot(), true); break;
                        case 3: enterShooting();                           break;
                    }
                }
                break;
        }
    }

    private void advance() { step++; enterStep(); }

    // ── PATHING ────────────────────────────────────────────────────────────────

    private void enterPathing(PathChain chain, boolean runIntake) {
        state = FsmState.PATHING;
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        intake.setPower(runIntake ? INTAKE_POWER : 0);
        hood.setPosition(SHOOT_HOOD_POS);
        shooter.setShooterVelocityRpm(SHOOT_RPM);
        runner.followPath(chain);
    }

    private void tickPathing() {
        shooter.setShooterVelocityRpm(SHOOT_RPM);
        if (!runner.isBusy()) {
            if (finalPath) state = FsmState.DONE;
            else advance();
        }
    }

    // ── SHOOTING ───────────────────────────────────────────────────────────────

    private void enterShooting() {
        state        = FsmState.SHOOTING;
        shooterFired = false;
        fireStartMs  = 0;
        intake.setPower(0);
        hood.setPosition(SHOOT_HOOD_POS);
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        shooter.setShooterVelocityRpm(SHOOT_RPM);
    }

    private void tickShooting() {
        long elapsed = System.currentTimeMillis() - stateEnteredMs;
        if (!shooterFired) {
            boolean atSpeed  = Math.abs(shooter.getShooterVelocityRpm() - SHOOT_RPM) < SHOOT_RPM_TOLERANCE;
            boolean timedOut = elapsed > SHOOT_SPINUP_TIMEOUT_MS;
            if (atSpeed || timedOut) {
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
        if (System.currentTimeMillis() - intakeWaitStart >= INTAKE_WAIT_MS) {
            intake.setPower(0);
            advance();
        }
    }

    // ── Path chains ────────────────────────────────────────────────────────────

    private PathChain chainApproach() {
        return follower.pathBuilder()
                .addPath(new BezierLine(START, SHOOT_START))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    private PathChain chainBall1() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_START, BALL1_SWEEP))
                .setConstantHeadingInterpolation(HEADING)
                .addPath(new BezierLine(BALL1_SWEEP, BALL1))
                .setConstantHeadingInterpolation(HEADING)
                .addPath(new BezierLine(BALL1, SHOOT))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    private PathChain chainBall2() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SHOOT, BALL2_SWEEP))
                .setConstantHeadingInterpolation(HEADING)
                .addPath(new BezierLine(BALL2_SWEEP, BALL2))
                .setConstantHeadingInterpolation(HEADING)
                .addPath(new BezierLine(BALL2, SHOOT))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    /** SHOOT → RELEASE (mirrored: heading 15° instead of 165°) */
    private PathChain chainShootToRelease() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SHOOT, RELEASE))
                .setConstantHeadingInterpolation(Math.toRadians(15))
                .build();
    }

    private PathChain chainReleaseToShoot() {
        return follower.pathBuilder()
                .addPath(new BezierLine(RELEASE, SHOOT))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    private PathChain chainFinal() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SHOOT, FINAL))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    // ── Telemetry ──────────────────────────────────────────────────────────────

    private void renderTelemetry() {
        Pose p = follower.getPose();
        telemetry.addData("State",  "%s (step %d)", state, step);
        telemetry.addData("Pose",   "(%.1f, %.1f) %.0f°",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("Turret", "%.1f deg  dist %.0f cm",
                shooter.getTurretAngleDeg(), shooter.getTrackedTagDistanceCm());
        telemetry.addData("Hood",   "%.3f", hood.getPosition());
        telemetry.addData("RPM",    "%.0f / %.0f  fired=%b",
                shooter.getShooterVelocityRpm(), SHOOT_RPM, shooterFired);
        if (state == FsmState.PATHING) {
            telemetry.addData("PathIdx", "%d", follower.getChainIndex());
        }
        telemetry.addData("LL pipe", shooter.getPipelineUploadStatus());
    }
}
