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
 *   step 0  PATHING   START(21,119) → SHOOT(53,88)                     [pre-spin, LL aim]
 *   step 1  SHOOTING  fire at SHOOT
 *   step 2  PATHING   SHOOT(53,88) → BALL1(7,83) → SHOOT(53,88)        [intake, pre-spin on return]
 *   step 3  SHOOTING  fire at SHOOT
 *   step 4  PATHING   SHOOT(53,88) → (39,59) → BALL2(9,59) → SHOOT     [intake, pre-spin on return]
 *   step 5  SHOOTING  fire at SHOOT
 */
@Config
@Autonomous(name = "Auto Blue Near", group = "Competition")
public class AutoBlueNear extends LinearOpMode {

    // ── Field poses (heading 180°: robot faces –X / backward) ─────────────────
    private static final double HEADING    = Math.toRadians(180);
    private static final Pose START        = new Pose(21.000, 119.000, HEADING);
    private static final Pose SHOOT        = new Pose(53.000,  88.000, HEADING);
    private static final Pose BALL1        = new Pose( 7.000,  83.000, HEADING);
    private static final Pose BALL2_SWEEP  = new Pose(39.000,  59.000, HEADING);
    private static final Pose BALL2        = new Pose( 9.000,  59.000, HEADING);

    // ── Shooter constants (tune from FTC Dashboard) ────────────────────────────
    public static double SHOOT_RPM            = 4000.0;
    public static double SHOOT_HOOD_POS       = 0.0;
    public static long   SHOOT_FIRE_MS        = 3000;
    public static double SHOOT_RPM_TOLERANCE  = 400.0;
    /** Max ms to wait for RPM after arriving — flywheel pre-spins during approach. */
    public static long   SHOOT_SPINUP_TIMEOUT_MS = 1500;

    // ── Intake ─────────────────────────────────────────────────────────────────
    public static double INTAKE_POWER = 1.0;

    // ── State machine ──────────────────────────────────────────────────────────
    private enum FsmState { PATHING, SHOOTING, DONE }

    private PedroAutoRunner  runner;
    private Follower         follower;
    private ShooterSubsystem shooter;
    private HoodSubsystem    hood;
    private IntakeSubsystem  intake;

    private FsmState state         = FsmState.PATHING;
    private int      step          = 0;
    private long     stateEnteredMs = 0;

    // PATHING sub-state
    private boolean intakeOnPath = false;
    // Which path index within the current chain is the first "return-to-SHOOT" leg.
    // When the follower reaches or passes this index we start pre-spinning.
    private int     preSpinPathIdx = 0;

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

        runner.setStartPose(START);

        // Command the hood servo to 0 while waiting for Start so it physically moves
        // before the match begins, not after.
        hood.setPosition(SHOOT_HOOD_POS);
        shooter.switchPipeline(ShooterConfig.APRILTAG_PIPELINE);

        telemetry.addLine("Auto Blue — waiting for start");
        telemetry.addData("SHOOT_RPM",      SHOOT_RPM);
        telemetry.addData("SHOOT_HOOD_POS", SHOOT_HOOD_POS);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

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
            shooter.runTurretControl(0, false); // LL AprilTag auto-aim every loop
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
            case 0: enterPathing(chainApproach(), false, 0);  break; // START → SHOOT (1 path, pre-spin from index 0)
            case 1: enterShooting();                           break;
            case 2: enterPathing(chainBall1(),    true,  1);  break; // SHOOT→BALL1→SHOOT (2 paths, pre-spin from index 1)
            case 3: enterShooting();                           break;
            case 4: enterPathing(chainBall2(),    true,  2);  break; // SHOOT→sweep→BALL2→SHOOT (3 paths, pre-spin from index 2)
            case 5: enterShooting();                           break;
            default: state = FsmState.DONE;                    break;
        }
    }

    private void advance() { step++; enterStep(); }

    // ── PATHING ────────────────────────────────────────────────────────────────

    /**
     * @param chain         path chain to follow
     * @param runIntake     true → intake runs the whole chain (ball collection paths)
     * @param preSpinAt     path index within the chain at which the flywheel starts pre-spinning.
     *                      Pass 0 to spin from the very first segment (approach to SHOOT).
     */
    private void enterPathing(PathChain chain, boolean runIntake, int preSpinAt) {
        state          = FsmState.PATHING;
        intakeOnPath   = runIntake;
        preSpinPathIdx = preSpinAt;
        shooter.switchPipeline(ShooterConfig.APRILTAG_PIPELINE);
        intake.setPower(runIntake ? INTAKE_POWER : 0);
        // Start pre-spinning immediately if preSpinAt == 0 (first path is already the approach).
        if (preSpinAt == 0) {
            hood.setPosition(SHOOT_HOOD_POS);
            shooter.setShooterVelocityRpm(SHOOT_RPM);
        } else {
            shooter.setShooterVelocityRpm(0);
        }
        runner.followPath(chain);
    }

    private void tickPathing() {
        // Start pre-spinning when the follower reaches the return-to-SHOOT leg.
        if (follower.getChainIndex() >= preSpinPathIdx) {
            hood.setPosition(SHOOT_HOOD_POS);
            shooter.setShooterVelocityRpm(SHOOT_RPM);
        }
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
                intake.setPower(INTAKE_POWER); // feed balls up through the shooter
                shooterFired = true;
                fireStartMs  = System.currentTimeMillis();
            }
        } else if (System.currentTimeMillis() - fireStartMs >= SHOOT_FIRE_MS) {
            shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
            intake.setPower(0);
            shooter.setShooterVelocityRpm(0);
            advance();
        }
    }

    // ── Path chains ────────────────────────────────────────────────────────────

    /** START → SHOOT */
    private PathChain chainApproach() {
        return follower.pathBuilder()
                .addPath(new BezierLine(START, SHOOT))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    /**
     * SHOOT → BALL1 → SHOOT
     * Return leg is reversed (robot drives backwards) so intake faces the same direction
     * throughout.
     */
    private PathChain chainBall1() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SHOOT, BALL1))
                .setConstantHeadingInterpolation(HEADING)
                .addPath(new BezierLine(BALL1, SHOOT))
                .setConstantHeadingInterpolation(HEADING)
                .setReversed()
                .build();
    }

    /** SHOOT → BALL2_SWEEP(39,59) → BALL2(9,59) → SHOOT */
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
            telemetry.addData("PathIdx", "%d  preSpinAt %d", follower.getChainIndex(), preSpinPathIdx);
        }
        telemetry.addData("LL pipe", shooter.getPipelineUploadStatus());
    }
}
