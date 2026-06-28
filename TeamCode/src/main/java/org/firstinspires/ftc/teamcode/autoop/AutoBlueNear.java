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
 * Blue-alliance autonomous routine.
 *
 * Path layout (all heading 180°):
 *   step 0  PATHING   START → SHOOT_START                               [shooter always on]
 *   step 1  SHOOTING  fire at SHOOT_START
 *   step 2  PATHING   SHOOT_START → BALL1_SWEEP → BALL1 → SHOOT         [intake]
 *   step 3  SHOOTING  fire at SHOOT
 *   step 4  PATHING   SHOOT → BALL2_SWEEP → BALL2 → SHOOT               [intake]
 *   step 5  SHOOTING  fire at SHOOT
 *   step 6  PATHING   SHOOT → FINAL                                      [park]
 */
@Config
@Autonomous(name = "Auto Blue Near", group = "Competition")
public class AutoBlueNear extends LinearOpMode {

    // ── Field poses (heading 180°: robot faces –X) ────────────────────────────
    private static final double HEADING    = Math.toRadians(180);
    private static final Pose START        = new Pose(21.000, 119.000, HEADING);
    private static final Pose SHOOT_START  = new Pose(53.000,  88.000, HEADING);
    private static final Pose BALL1_SWEEP  = new Pose(39.000,  77.000, HEADING);
    private static final Pose BALL1        = new Pose( 9.000,  77.000, HEADING);
    private static final Pose BALL2_SWEEP  = new Pose(39.000,  44.000, HEADING);
    private static final Pose BALL2        = new Pose( 8.000,  44.000, HEADING);
    private static final Pose FINAL        = new Pose( 10.00,  88.000, HEADING);
    private static final Pose SHOOT        = new Pose(50.000,  83.000, HEADING);
    private static final Pose RELEASE      = new Pose(11.500,  61.500, 165);

    // ── Shooter constants (tune from FTC Dashboard) ────────────────────────────
    public static double SHOOT_RPM            = 4250.0;
    public static double SHOOT_HOOD_POS       = 0.0;
    public static long   SHOOT_FIRE_MS        = 1500;
    public static double SHOOT_RPM_TOLERANCE  = 400.0;
    /** Safety-net wait after arriving — shooter is already at speed so this rarely triggers. */
    public static long   SHOOT_SPINUP_TIMEOUT_MS = 0;

    // ── Intake ─────────────────────────────────────────────────────────────────
    public static double INTAKE_POWER     = 1.0;
    public static long   INTAKE_WAIT_MS   = 3000;

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

    // SHOOTING sub-state
    private boolean shooterFired    = false;
    private long    fireStartMs     = 0;

    // INTAKE_WAIT sub-state
    private long    intakeWaitStart = 0;

    @Override
    public void runOpMode() {
        runner   = new PedroAutoRunner(hardwareMap);
        follower = runner.getFollower();
        shooter  = new ShooterSubsystem(hardwareMap);
        hood     = new HoodSubsystem(hardwareMap);
        intake   = new IntakeSubsystem(hardwareMap);

        runner.setStartPose(START);
        hood.setPosition(SHOOT_HOOD_POS);

        telemetry.addLine("Auto Blue Near — waiting for start");
        telemetry.addData("SHOOT_RPM",      SHOOT_RPM);
        telemetry.addData("SHOOT_HOOD_POS", SHOOT_HOOD_POS);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Spin up immediately — the flywheel runs the entire auto.
        shooter.setShooterVelocityRpm(SHOOT_RPM);
        enterStep();

        while (opModeIsActive() && !isStopRequested()) {
            switch (state) {
                case PATHING:     tickPathing();     break;
                case SHOOTING:    tickShooting();    break;
                case INTAKE_WAIT: tickIntakeWait();  break;
                case DONE:                           break;
            }

            follower.update();
            shooter.setTurretPower(0);
            shooter.updatePID();
            renderTelemetry();
            telemetry.update();
        }

        intake.setPower(0);
        shooter.setShooterVelocityRpm(0);
    }

    // ── Step sequencer ─────────────────────────────────────────────────────────

    private void enterStep() {
        stateEnteredMs = System.currentTimeMillis();
        switch (step) {
            case 0:  enterPathing(chainApproach(),      false); break;
            case 1:  enterShooting();                           break;
            case 2:  enterPathing(chainBall1(),          true); break;
            case 3:  enterShooting();                           break;
            case 4:  enterPathing(chainBall2(),          true); break;
            case 5:  enterShooting();                           break;
            // RELEASE cycle 1
            case 6:  enterPathing(chainShootToRelease(), true); break;
            case 7:  enterIntakeWait();                         break;
            case 8:  enterPathing(chainReleaseToShoot(), false);break;
            case 9:  enterShooting();                           break;
            // RELEASE cycle 2
            case 10: enterPathing(chainShootToRelease(), true); break;
            case 11: enterIntakeWait();                         break;
            case 12: enterPathing(chainReleaseToShoot(), false);break;
            case 13: enterShooting();                           break;
            // Park
            case 14: enterPathing(chainFinal(),          false);break;
            default: state = FsmState.DONE;                     break;
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
        if (!runner.isBusy()) advance();
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
            advance(); // shooter stays at SHOOT_RPM — enterPathing keeps it running
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
        if (System.currentTimeMillis() - intakeWaitStart >= INTAKE_WAIT_MS) {
            intake.setPower(0);
            advance();
        }
    }

    // ── Path chains ────────────────────────────────────────────────────────────

    /** START → SHOOT_START */
    private PathChain chainApproach() {
        return follower.pathBuilder()
                .addPath(new BezierLine(START, SHOOT_START))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    /** SHOOT_START → BALL1_SWEEP → BALL1 → SHOOT */
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

    /** SHOOT → BALL2_SWEEP → BALL2 → SHOOT */
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

    /** SHOOT → RELEASE */
    private PathChain chainShootToRelease() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SHOOT, RELEASE))
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();
    }

    /** RELEASE → SHOOT */
    private PathChain chainReleaseToShoot() {
        return follower.pathBuilder()
                .addPath(new BezierLine(RELEASE, SHOOT))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    /** SHOOT → FINAL (park) */
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
