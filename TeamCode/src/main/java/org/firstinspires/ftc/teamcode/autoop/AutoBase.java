package org.firstinspires.ftc.teamcode.autoop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.tools.localization.PanelsFieldDrawer;

/**
 * Base class for every PedroPathing auto. Handles all the boilerplate — building
 * the follower, optional AprilTag correction, the update loop, and telemetry — so
 * a new auto is just: pick a start pose, build some paths, react to events.
 *
 * To write a new auto, extend this and implement {@link #startPose()} and
 * {@link #onStart()}. The protected {@link #runner} and {@link #follower} are ready
 * to use inside those methods. See {@link AutoBlueLeft} for a worked example.
 *
 * To also fuse AprilTag corrections, override {@link #createShooterForCorrection()}
 * (and the tag getters) to return a configured shooter.
 */
public abstract class AutoBase extends LinearOpMode {

    protected PedroAutoRunner runner;
    protected Follower follower;

    /** Robot's starting field pose (Pedro coordinates: inches, heading in radians). */
    protected abstract Pose startPose();

    /** Runs once right after START — build paths and kick off the routine here. */
    protected abstract void onStart();

    /** Optional per-loop hook, e.g. spin up a shooter while a path runs. */
    protected void onLoop() {}

    // --- Optional AprilTag correction wiring (return null to disable) ---
    protected ShooterSubsystem createShooterForCorrection() { return null; }
    protected int correctionTagId() { return 0; }
    protected double correctionTagX() { return 0; }
    protected double correctionTagY() { return 0; }

    @Override
    public void runOpMode() {
        runner = new PedroAutoRunner(hardwareMap);
        follower = runner.getFollower();
        runner.setStartPose(startPose());

        ShooterSubsystem shooter = createShooterForCorrection();
        if (shooter != null) {
            runner.enableAprilTagCorrection(shooter, correctionTagId(),
                    correctionTagX(), correctionTagY());
        }

        PanelsFieldDrawer.init();

        telemetry.addLine("Pedro auto initialized. Press START.");
        Pose s = startPose();
        telemetry.addData("Start", "(%.1f, %.1f, %.1f deg)",
                s.getX(), s.getY(), Math.toDegrees(s.getHeading()));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        onStart();

        while (opModeIsActive() && !isStopRequested()) {
            runner.update();
            onLoop();
            PanelsFieldDrawer.update(follower);
            renderTelemetry();
            telemetry.update();
        }
        runner.stop();
    }

    private void renderTelemetry() {
        Pose p = follower.getPose();
        telemetry.addData("Following", runner.isBusy());
        telemetry.addData("X", "%.1f", p.getX());
        telemetry.addData("Y", "%.1f", p.getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(p.getHeading()));
        if (PedroAutoRunner.USE_APRILTAG_CORRECTION) {
            telemetry.addData("Tag visible", runner.getLastObservation().visible);
            telemetry.addData("Tag dist", "%.1f in", runner.getLastObservation().distanceIn);
        }
    }
}
