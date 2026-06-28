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
 * Test routine for the RELEASE pose.
 *
 *   step 0  PATHING      START → SHOOT_START
 *   step 1  PATHING      SHOOT_START → RELEASE   [intake on]
 *   step 2  INTAKE_WAIT  dwell at RELEASE for INTAKE_WAIT_MS
 *   step 3  PATHING      RELEASE → SHOOT
 *   step 4  SHOOTING     fire at SHOOT
 */
@Config
@Autonomous(name = "Test Release Pose", group = "Test")
public class TestRelease extends LinearOpMode {

    private static final double HEADING     = Math.toRadians(180);
    private static final Pose   START       = new Pose(21.000, 119.000, HEADING);
    private static final Pose   SHOOT_START = new Pose(53.000,  88.000, HEADING);
    private static final Pose   SHOOT       = new Pose(50.000,  83.000, HEADING);
    private static final Pose   RELEASE     = new Pose(11.500,  61.500, 165);

    public static double SHOOT_RPM           = 4250.0;
    public static double SHOOT_HOOD_POS      = 0.0;
    public static long   SHOOT_FIRE_MS       = 1500;
    public static double SHOOT_RPM_TOLERANCE = 400.0;
    public static long   SHOOT_SPINUP_TIMEOUT_MS = 0;
    public static double INTAKE_POWER        = 1.0;
    public static long   INTAKE_WAIT_MS      = 3000;

    private enum FsmState { PATHING, INTAKE_WAIT, SHOOTING, DONE }

    private PedroAutoRunner  runner;
    private Follower         follower;
    private ShooterSubsystem shooter;
    private HoodSubsystem    hood;
    private IntakeSubsystem  intake;

    private FsmState state          = FsmState.PATHING;
    private int      step           = 0;
    private long     stateEnteredMs = 0;

    private boolean shooterFired    = false;
    private long    fireStartMs     = 0;
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

        telemetry.addLine("Test Release Pose — waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        shooter.setShooterVelocityRpm(SHOOT_RPM);
        enterStep();

        while (opModeIsActive() && !isStopRequested()) {
            switch (state) {
                case PATHING:     tickPathing();    break;
                case INTAKE_WAIT: tickIntakeWait(); break;
                case SHOOTING:    tickShooting();   break;
                case DONE:                          break;
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

    private void enterStep() {
        stateEnteredMs = System.currentTimeMillis();
        switch (step) {
            case 0: enterPathing(chainApproach(),        false); break;
            case 1: enterPathing(chainShootToRelease(),  true);  break;
            case 2: enterIntakeWait();                           break;
            case 3: enterPathing(chainReleaseToShoot(),  false); break;
            case 4: enterShooting();                             break;
            default: state = FsmState.DONE;                      break;
        }
    }

    private void advance() { step++; enterStep(); }

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

    /** START → SHOOT_START */
    private PathChain chainApproach() {
        return follower.pathBuilder()
                .addPath(new BezierLine(START, SHOOT_START))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    /** SHOOT_START → RELEASE */
    private PathChain chainShootToRelease() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_START, RELEASE))
                .setConstantHeadingInterpolation(Math.toRadians(165))
                .build();
    }

    /** RELEASE → SHOOT */
    private PathChain chainReleaseToShoot() {
        return follower.pathBuilder()
                .addPath(new BezierLine(RELEASE, SHOOT))
                .setConstantHeadingInterpolation(HEADING)
                .build();
    }

    private void renderTelemetry() {
        Pose p = follower.getPose();
        telemetry.addData("State",  "%s (step %d)", state, step);
        telemetry.addData("Pose",   "(%.1f, %.1f) %.0f°",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("RPM",    "%.0f / %.0f  fired=%b",
                shooter.getShooterVelocityRpm(), SHOOT_RPM, shooterFired);
        if (state == FsmState.INTAKE_WAIT) {
            telemetry.addData("IntakeWait", "%d ms remaining",
                    INTAKE_WAIT_MS - (System.currentTimeMillis() - intakeWaitStart));
        }
        if (state == FsmState.PATHING) {
            telemetry.addData("PathIdx", "%d", follower.getChainIndex());
        }
    }
}
