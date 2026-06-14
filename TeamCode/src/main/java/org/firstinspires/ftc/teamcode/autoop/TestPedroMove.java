package org.firstinspires.ftc.teamcode.autoop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test Pedro Move", group = "Pedro")
public class TestPedroMove extends AutoBase {

    private Paths paths;

    @Override
    protected Pose startPose() {
        return new Pose(53.000, 88.000, Math.toRadians(180));
    }

    @Override
    protected void onStart() {
        paths = new Paths(follower);
        runner.followPath(paths.BlueTop);
    }

    public static class Paths {
        public PathChain BlueTop;

        public Paths(Follower follower) {
            BlueTop = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(53.000, 88.000),
                                    new Pose(17.000, 82.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(17.000, 82.000),
                                    new Pose(53.000, 88.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .setReversed()
                    .addPath(
                            new BezierLine(
                                    new Pose(53.000, 88.000),
                                    new Pose(42.000, 59.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(42.000, 59.000),
                                    new Pose(19.000, 59.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(19.000, 59.000),
                                    new Pose(53.000, 88.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(53.000, 88.000),
                                    new Pose(53.000, 57.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}
