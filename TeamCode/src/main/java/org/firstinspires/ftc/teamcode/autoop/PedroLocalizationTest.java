package org.firstinspires.ftc.teamcode.autoop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tools.localization.PanelsFieldDrawer;

/**
 * THE FIRST THING TO RUN when bringing up PedroPathing.
 *
 * Drive the robot around (or push it by hand) and watch X / Y / Heading. This is
 * how you verify and tune the localizer in {@link PedroConstants} before trusting
 * any path:
 *
 *   1. Push the robot exactly one tile (24") FORWARD. X should read ~24, Y ~0.
 *      If X goes negative, flip the encoder direction signs in PedroConstants.
 *      If the magnitude is off, scale FORWARD_TICKS_TO_INCHES.
 *   2. Push it one tile LEFT. Y should read ~24, X ~0. Fix STRAFE_TICKS_TO_INCHES
 *      / signs the same way.
 *   3. Spin the robot 360 deg. Heading should return to ~0. Tune TURN_TICKS_TO_INCHES.
 *
 * All those constants are live-tunable from FTC Dashboard / Panels, so you can
 * adjust and re-test without redeploying.
 *
 * Full tuning guide: https://pedropathing.com/docs/pathing
 */
@TeleOp(name = "Pedro Localization Test", group = "Pedro")
public class PedroLocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Pedro Localization Test ready.");
        telemetry.addLine("After START: drive with sticks, or push the robot by hand.");
        telemetry.addLine("Watch X / Y / Heading to calibrate PedroConstants.");
        telemetry.update();

        PanelsFieldDrawer.init();
        waitForStart();
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            // Left stick = translate, right stick X = turn. Robot-centric drive.
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            follower.setTeleOpDrive(forward, strafe, turn, true);
            follower.update();

            // Draw robot + trail on the Panels field view.
            PanelsFieldDrawer.update(follower);

            Pose pose = follower.getPose();
            telemetry.addData("X (in)", "%.2f", pose.getX());
            telemetry.addData("Y (in)", "%.2f", pose.getY());
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.getHeading()));
            telemetry.addLine("--- calibration reminders ---");
            telemetry.addLine("Push 24\" fwd -> X~24, Y~0 | Push 24\" left -> Y~24, X~0");
            telemetry.update();
        }
    }
}
