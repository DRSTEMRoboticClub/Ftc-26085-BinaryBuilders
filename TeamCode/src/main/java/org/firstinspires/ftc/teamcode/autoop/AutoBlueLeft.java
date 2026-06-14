package org.firstinspires.ftc.teamcode.autoop;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Example Blue-alliance auto and the template for writing your own.
 *
 * To make a new auto: copy this file, rename the class + {@code @Autonomous} name,
 * change {@link #startPose()}, and edit {@link #onStart()} to build your paths.
 * Everything else (follower setup, update loop, telemetry) is handled by
 * {@link AutoBase}.
 *
 * Paths are built with {@code follower.pathBuilder()}. A {@link BezierLine} is a
 * straight segment; use {@code BezierCurve} for curves. Heading control options:
 *   - setConstantHeadingInterpolation(rad)         : hold a fixed heading
 *   - setLinearHeadingInterpolation(start, end)    : rotate smoothly along the path
 *   - setTangentHeadingInterpolation()             : face the direction of travel
 */
@Autonomous(name = "Blue Left Auto", group = "Pedro")
public class AutoBlueLeft extends AutoBase {

    @Override
    protected Pose startPose() {
        // Set to wherever the robot actually starts (Pedro coords, heading radians).
        return new Pose(0, 0, 0);
    }

    @Override
    protected void onStart() {
        // Two-segment example: drive forward 24", then strafe-and-turn to (24,24,90deg).
        PathChain routine = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0), new Pose(24, 0)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Pose(24, 0), new Pose(24, 24)))
                .setLinearHeadingInterpolation(0, Math.toRadians(90))
                // Example: fire a callback partway through (e.g. start a shooter).
                // .addTemporalCallback(0.5, () -> shooter.setShooterPower(0.8))
                .build();

        runner.followPath(routine);
    }

    // Example of running a subsystem alongside path following — uncomment and wire
    // up a ShooterSubsystem field if you need it:
    //
    // @Override protected void onLoop() {
    //     if (!runner.isBusy()) shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
    // }
}
