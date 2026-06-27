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
 *   step 1  PATHING   (68,6)→(39,35)→(8,35)→(68,6)   [intake, pre-spin on return at idx 2, reversed]
 *   step 2  SHOOTING  fire at (68,6)
 *   step 3  PATHING   (68,6)→(1,8)→(68,6)             [intake, pre-spin on return at idx 1]
 *   step 4  SHOOTING  fire at (68,6)
 */
@Config
@Autonomous(name = "Auto Blue Far", group = "Competition")
public class AutoBlueFar extends LinearOpMode {

    // ── Field poses (heading 180°) ─────────────────────────────────────────────
    private static final double HEADING      = Math.toRadians(180);
    private static final Pose START_SHOOT    = new Pose(68.000,  6.000, HEADING);
    private static final Pose BALL1_MID      = new Pose(39.000, 35.000, HEADING);
    private static final Pose BALL1          = new Pose( 8.000, 35.000, HEADING);
    private static final Pose BALL2          = new Pose( 1.000,  8.000, HEADING);

    // ── Shooter constants (tune from FTC Dashboard) ────────────────────────────
    public static double SHOOT_RPM            = 4000.0;
    public static double SHOOT_HOOD_POS       = 0.0;
    public static long   SHOOT_FIRE_MS        = 3000;
    public static double SHOOT_RPM_TOLERANCE  = 400.0;
    /** Safety-net wait after arriving — shooter is already at speed so this rarely triggers. */
    public static long   SHOOT_SPINUP_TIMEOUT_MS = 500;

    // ── Intake ─────────────────────────────────────────────────────────────────
    public static double INTAKE_POWER = 1.0;

    // ── State machine ──────────────────────────────────────────────────────────
    private enum FsmState { PATHING, SHOOTING, DONE }

    private PedroAutoRunner  runner;
    private Follower         follower;
    private ShooterSubsystem shooter;
    private HoodSubsystem    hood;
    private IntakeSubsystem  intake;

    private FsmState state          = FsmState.PATHING;
    private int      step           = 0;
    private long     stateEnteredMs = 0;

    // SHOOTING sub-state
    private boolean shooterFired = false;
    private long    fireStartMs  = 0;

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
            shooter.cacheLimelightResult();
            shooter.setRobotHeading(Math.toDegrees(follower.getPose().getHeading()));

            switch (state) {
                case PATHING:  tickPathing();  break;
                case SHOOTING: tickShooting(); break;
                case DONE:                     break;
            }

            follower.update();
            shooter.runTurretControl(0, false);
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
            case 0: enterShooting();                     break;
            case 1: enterPathing(chainBall1(), true);    break;
            case 2: enterShooting();                     break;
            case 3: enterPathing(chainBall2(), true);    break;
            case 4: enterShooting();                     break;
            default: state = FsmState.DONE;              break;
        }
    }

    private void advance() { step++; enterStep(); }

    // ── PATHING ────────────────────────────────────────────────────────────────

    private void enterPathing(PathChain chain, boolean runIntake) {
        state = FsmState.PATHING;
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED); // ensure closed before moving
        shooter.switchPipeline(ShooterConfig.APRILTAG_PIPELINE);
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
        shooter.switchPipeline(ShooterConfig.APRILTAG_PIPELINE);
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

    // ── Path chains ────────────────────────────────────────────────────────────

    /** (68,6) → (39,35) → (8,35) → (68,6) */
    private PathChain chainBall1() {
        return follower.pathBuilder()
                .addPath(new BezierLine(START_SHOOT, BALL1_MID))
                .setConstantHeadingInterpolation(HEADING)
                .addPath(new BezierLine(BALL1_MID, BALL1))
                .setConstantHeadingInterpolation(HEADING)
                .addPath(new BezierLine(BALL1, START_SHOOT))
                .setConstantHeadingInterpolation(HEADING)
                .setReversed()
                .build();
    }

    /** (68,6) → (1,8) → (68,6) */
    private PathChain chainBall2() {
        return follower.pathBuilder()
                .addPath(new BezierLine(START_SHOOT, BALL2))
                .setConstantHeadingInterpolation(HEADING)
                .addPath(new BezierLine(BALL2, START_SHOOT))
                .setConstantHeadingInterpolation(HEADING)
                .setReversed()
                .build();
    }

    // ── Telemetry ──────────────────────────────────────────────────────────────

    private void renderTelemetry() {
        Pose p = follower.getPose();
        telemetry.addData("State",   "%s (step %d)", state, step);
        telemetry.addData("Pose",    "(%.1f, %.1f) %.0f°",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("Turret",  "%.1f deg  dist %.0f cm",
                shooter.getTurretAngleDeg(), shooter.getTrackedTagDistanceCm());
        telemetry.addData("Hood",    "%.3f", hood.getPosition());
        telemetry.addData("RPM",     "%.0f / %.0f  fired=%b",
                shooter.getShooterVelocityRpm(), SHOOT_RPM, shooterFired);
        if (state == FsmState.PATHING) {
            telemetry.addData("PathIdx", "%d", follower.getChainIndex());
        }
    }
}
