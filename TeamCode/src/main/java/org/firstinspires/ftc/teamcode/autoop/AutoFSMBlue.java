package org.firstinspires.ftc.teamcode.autoop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.teleop.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.ShooterSubsystem;

import java.util.List;

/**
 * Three-state FSM autonomous routine for Blue alliance.
 *
 * States:
 *   PATHING  — Pedro Pathing follows a pre-built path to a waypoint.
 *   INTAKING — Intake motor runs; Limelight colour-blob pipeline steers
 *              the whole robot toward the nearest ball until it is close
 *              enough for the intake to grab it (or a timeout fires).
 *   SHOOTING — Shooter spins to target RPM, stopper opens to fire,
 *              then closes and shooter spins down.
 *
 * Sequence (step index):
 *   0  PATHING  → ball 1 pickup zone
 *   1  INTAKING → acquire ball 1
 *   2  PATHING  → shoot zone 1
 *   3  SHOOTING → fire ball 1
 *   4  PATHING  → ball 2 pickup zone
 *   5  INTAKING → acquire ball 2
 *   6  PATHING  → shoot zone 2
 *   7  SHOOTING → fire ball 2
 *   8  DONE
 *
 * All tuning constants are live-editable from FTC Dashboard / Panels.
 */
@Config
@Autonomous(name = "FSM Auto Blue", group = "Production")
public class AutoFSMBlue extends LinearOpMode {

    // ── Vision / intake tuning ────────────────────────────────────────────
    /** Limelight pipeline index configured for colour-blob (ball) detection. */
    public static int    COLOR_PIPELINE          = 1;
    /** Horizontal tx must be within this many degrees before we drive forward. */
    public static double INTAKE_TX_DEADBAND_DEG  = 6.0;
    /** Proportional turn gain: power-per-degree of tx error (positive tx → right). */
    public static double INTAKE_TURN_GAIN        = 0.025;
    /** Forward speed while driving toward a centred ball. */
    public static double INTAKE_DRIVE_POWER      = 0.30;
    /** Limelight colour-blob area % at which ball is considered at the intake. */
    public static double INTAKE_AREA_THRESHOLD   = 3.0;
    /** Give up on ball acquisition after this many milliseconds. */
    public static long   INTAKE_TIMEOUT_MS       = 5000;

    // ── Shooter tuning ────────────────────────────────────────────────────
    /** RPM window around target before we consider the shooter ready to fire. */
    public static double SHOOT_RPM_TOLERANCE      = 200.0;
    /** Fire anyway after this many ms if RPM never converges. */
    public static long   SHOOT_SPINUP_TIMEOUT_MS  = 3000;
    /** How long the stopper stays open while firing (ms). */
    public static long   SHOOT_FIRE_MS            = 600;

    // ── State machine ─────────────────────────────────────────────────────

    private enum FsmState { PATHING, INTAKING, SHOOTING, DONE }

    private PedroAutoRunner  runner;
    private Follower         follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem  intake;
    private Paths            paths;

    private FsmState state          = FsmState.PATHING;
    private int      step           = 0;
    private long     stateEnteredMs = 0;

    // SHOOTING sub-state
    private boolean shooterFired = false;
    private long    fireStartMs  = 0;

    // ── runOpMode ─────────────────────────────────────────────────────────

    @Override
    public void runOpMode() {
        runner   = new PedroAutoRunner(hardwareMap);
        follower = runner.getFollower();
        shooter  = new ShooterSubsystem(hardwareMap);
        intake   = new IntakeSubsystem(hardwareMap);
        paths    = new Paths();

        runner.setStartPose(paths.START);

        telemetry.addLine("FSM Auto Blue — waiting for start");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        enterStep();

        while (opModeIsActive() && !isStopRequested()) {
            switch (state) {
                case PATHING:  tickPathing();  break;
                case INTAKING: tickIntaking(); break;
                case SHOOTING: tickShooting(); break;
                case DONE:                     break;
            }
            follower.update();
            shooter.updatePID(); // must run every loop to drive the flywheel PID
            renderTelemetry();
            telemetry.update();
        }

        intake.setPower(0);
        shooter.setShooterVelocityRpm(0);
        shooter.stopLimelight();
    }

    // ── Step sequencer ────────────────────────────────────────────────────

    private void enterStep() {
        stateEnteredMs = System.currentTimeMillis();
        switch (step) {
            case 0:  enterPathing(paths.toBall1);   break;
            case 1:  enterIntaking();               break;
            case 2:  enterPathing(paths.toShoot1);  break;
            case 3:  enterShooting();               break;
            case 4:  enterPathing(paths.toBall2);   break;
            case 5:  enterIntaking();               break;
            case 6:  enterPathing(paths.toShoot2);  break;
            case 7:  enterShooting();               break;
            default: state = FsmState.DONE;         break;
        }
    }

    private void advance() {
        step++;
        enterStep();
    }

    // ── PATHING ───────────────────────────────────────────────────────────

    private void enterPathing(Path path) {
        state = FsmState.PATHING;
        intake.setPower(0);
        shooter.switchPipeline(ShooterConfig.APRILTAG_PIPELINE);
        runner.followPath(path);
    }

    private void tickPathing() {
        if (!runner.isBusy()) advance();
    }

    // ── INTAKING ──────────────────────────────────────────────────────────

    private void enterIntaking() {
        state = FsmState.INTAKING;
        intake.setPower(1.0);
        shooter.switchPipeline(COLOR_PIPELINE);
        // Switch Pedro to reactive drive so we can steer with setTeleOpDrive().
        follower.startTeleopDrive();
    }

    private void tickIntaking() {
        // Timeout: give up and advance even without a ball.
        if (System.currentTimeMillis() - stateEnteredMs > INTAKE_TIMEOUT_MS) {
            finishIntaking();
            return;
        }

        LLResult result = shooter.getLimelightResult();
        if (result == null) {
            follower.setTeleOpDrive(0, 0, 0, true);
            return;
        }

        List<LLResultTypes.ColorResult> blobs = result.getColorResults();
        if (blobs == null || blobs.isEmpty()) {
            follower.setTeleOpDrive(0, 0, 0, true);
            return;
        }

        // Pick the largest blob — largest area = nearest ball.
        LLResultTypes.ColorResult best = blobs.get(0);
        for (LLResultTypes.ColorResult b : blobs) {
            if (b.getTargetArea() > best.getTargetArea()) best = b;
        }

        double tx   = best.getTargetXDegrees();
        double area = best.getTargetArea();

        // Ball has reached the intake mouth — done.
        if (area >= INTAKE_AREA_THRESHOLD) {
            finishIntaking();
            return;
        }

        // Proportional turn to centre the ball in camera view (tx > 0 → ball right → turn right).
        // Only drive forward once the ball is close enough to centre.
        double turn    = -tx * INTAKE_TURN_GAIN;
        double forward = (Math.abs(tx) <= INTAKE_TX_DEADBAND_DEG) ? INTAKE_DRIVE_POWER : 0;
        follower.setTeleOpDrive(forward, 0, turn, true);
    }

    private void finishIntaking() {
        intake.setPower(0);
        follower.breakFollowing();
        advance();
    }

    // ── SHOOTING ──────────────────────────────────────────────────────────

    private void enterShooting() {
        state        = FsmState.SHOOTING;
        shooterFired = false;
        fireStartMs  = 0;
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        shooter.setShooterVelocityRpm(ShooterConfig.MANUAL_TARGET_RPM);
    }

    private void tickShooting() {
        long elapsed = System.currentTimeMillis() - stateEnteredMs;

        if (!shooterFired) {
            double actual  = shooter.getShooterVelocityRpm();
            double target  = ShooterConfig.MANUAL_TARGET_RPM;
            boolean atSpeed  = Math.abs(actual - target) < SHOOT_RPM_TOLERANCE;
            boolean timedOut = elapsed > SHOOT_SPINUP_TIMEOUT_MS;

            if (atSpeed || timedOut) {
                shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
                shooterFired = true;
                fireStartMs  = System.currentTimeMillis();
            }
        } else {
            if (System.currentTimeMillis() - fireStartMs >= SHOOT_FIRE_MS) {
                shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
                shooter.setShooterVelocityRpm(0);
                advance();
            }
        }
    }

    // ── Telemetry ─────────────────────────────────────────────────────────

    private void renderTelemetry() {
        Pose p = follower.getPose();
        telemetry.addData("State",  "%s (step %d)", state, step);
        telemetry.addData("Pose",   "(%.1f, %.1f) %.1f°",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));

        if (state == FsmState.INTAKING) {
            LLResult r = shooter.getLimelightResult();
            if (r != null) {
                List<LLResultTypes.ColorResult> blobs = r.getColorResults();
                telemetry.addData("Blobs seen", blobs != null ? blobs.size() : 0);
                if (blobs != null && !blobs.isEmpty()) {
                    LLResultTypes.ColorResult best = blobs.get(0);
                    for (LLResultTypes.ColorResult b : blobs)
                        if (b.getTargetArea() > best.getTargetArea()) best = b;
                    telemetry.addData("Best TX",   "%.1f°", best.getTargetXDegrees());
                    telemetry.addData("Best Area", "%.2f%%", best.getTargetArea());
                }
            } else {
                telemetry.addData("Limelight", "no result");
            }
            telemetry.addData("Intake timeout", "%d / %d ms",
                    System.currentTimeMillis() - stateEnteredMs, INTAKE_TIMEOUT_MS);
        }

        if (state == FsmState.SHOOTING) {
            telemetry.addData("Shooter RPM", "%.0f / %.0f",
                    shooter.getShooterVelocityRpm(), ShooterConfig.MANUAL_TARGET_RPM);
            telemetry.addData("Fired", shooterFired);
        }
    }

    // ── Paths (tune coordinates before match) ────────────────────────────

    static class Paths {
        // TODO: measure and tune all poses for your actual field layout.
        final Pose START   = new Pose(53, 88, Math.toRadians(180));

        final Path toBall1;
        final Path toShoot1;
        final Path toBall2;
        final Path toShoot2;

        Paths() {
            toBall1  = line(new Pose(53, 88,  Math.toRadians(180)),
                            new Pose(17, 82,  Math.toRadians(180)));
            toShoot1 = line(new Pose(17, 82,  Math.toRadians(180)),
                            new Pose(53, 88,  Math.toRadians(180)));
            toBall2  = line(new Pose(53, 88,  Math.toRadians(180)),
                            new Pose(42, 59,  Math.toRadians(180)));
            toShoot2 = line(new Pose(42, 59,  Math.toRadians(180)),
                            new Pose(53, 88,  Math.toRadians(180)));
        }

        private static Path line(Pose from, Pose to) {
            return new Path(new BezierLine(from, to));
        }
    }
}
