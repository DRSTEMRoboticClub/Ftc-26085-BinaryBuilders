package org.firstinspires.ftc.teamcode.autoop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Passive localizer check — drives NOTHING, so it is safe to push the robot by hand.
 *
 * Use this to fix "the robot rams the wall instead of driving to the target": that is the
 * follower getting inverted odometry feedback. Pedro must see the pose move the RIGHT way
 * before any path will work. Run this, push the robot, and confirm:
 *
 *   Push the robot AWAY from you (field +X)  → X should INCREASE.
 *   Push the robot to its LEFT  (field +Y)   → Y should INCREASE.
 *   Rotate the robot COUNTER-CLOCKWISE       → Heading should INCREASE.
 *
 * If any of those goes the wrong way, flip the matching encoder direction(s) in
 * PedroConstants from FTC Dashboard (LIVE — no redeploy needed):
 *   LF_ENCODER_DIR, RF_ENCODER_DIR, LR_ENCODER_DIR, RR_ENCODER_DIR  (+1 / -1)
 *
 * Tuning order (push the robot a known distance, e.g. one 61 cm tile, each time):
 *   1. Forward wrong way  → the X axis is inverted.
 *   2. Strafe wrong way   → the Y axis is inverted.
 *   3. Rotation wrong way → heading is inverted.
 * Flip signs until all three read correctly AND the reported distance matches reality
 * (scale FORWARD/STRAFE/TURN_TICKS_TO_INCHES for magnitude once directions are right).
 *
 * Once every direction is correct here, the path autos (TestAuto, etc.) will drive toward
 * their targets instead of away from them.
 */
@TeleOp(name = "Pedro Localization Test", group = "Pedro")
public class PedroLocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Localizer test — push the robot by hand (no motors run).");
        telemetry.addLine("Forward -> X up | Left -> Y up | CCW -> Heading up");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update(); // updates localization only; no path => no motor power

            Pose p = follower.getPose();
            telemetry.addData("X (in)",       "%.2f", p.getX());
            telemetry.addData("Y (in)",       "%.2f", p.getY());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(p.getHeading()));
            telemetry.addLine("Push fwd: X up | strafe left: Y up | turn CCW: heading up");
            telemetry.addLine("Wrong way? Flip LF/RF/LR/RR_ENCODER_DIR on Dashboard.");
            telemetry.update();
        }
    }
}
