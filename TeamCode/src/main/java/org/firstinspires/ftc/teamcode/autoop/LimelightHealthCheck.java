package org.firstinspires.ftc.teamcode.autoop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.HardwareConfig;

import java.util.List;

/**
 * Standalone Limelight diagnostic — no other subsystems, no ShooterSubsystem.
 *
 * Run this BEFORE deploying anything else to confirm the camera link is alive.
 *
 * Controls (gamepad 1):
 *   A / B / X / Y  — switch to pipeline 0 / 1 / 2 / 3
 *   Right bumper   — send updateRobotOrientation(0) this frame (toggle to test MegaTag2)
 *   Left  bumper   — reset frame counter
 */
@TeleOp(name = "LL Health Check", group = "Debug")
public class LimelightHealthCheck extends LinearOpMode {

    @Override
    public void runOpMode() {

        // ── 1. Hardware-map lookup (catch rather than crash on missing name) ──
        Limelight3A ll = null;
        String findStatus;
        try {
            ll = hardwareMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT_NAME);
            findStatus = "FOUND as \"" + HardwareConfig.LIMELIGHT_NAME + "\"";
        } catch (Exception e) {
            findStatus = "NOT FOUND (\"" + HardwareConfig.LIMELIGHT_NAME
                    + "\") — check robot config name matches HardwareConfig.LIMELIGHT_NAME";
        }

        telemetry.addData("HW Map", findStatus);

        if (ll == null) {
            telemetry.addLine("Cannot continue — fix the hardware-map name first.");
            telemetry.update();
            waitForStart();
            return;
        }

        // ── 2. Start on pipeline 0 ──────────────────────────────────────────
        ll.pipelineSwitch(0);
        ll.start();

        telemetry.addData("HW Map",  findStatus);
        telemetry.addData("Status",  "Started on pipeline 0. Press START.");
        telemetry.addLine("A=pipeline0  B=pipeline1  X=pipeline2  Y=pipeline3");
        telemetry.update();

        waitForStart();

        // ── 3. Diagnostic loop ──────────────────────────────────────────────
        int  currentPipeline = 0;
        long totalFrames     = 0;
        long validFrames     = 0;
        long nonNullFrames   = 0;

        boolean prevA = false, prevB = false, prevX = false, prevY = false;
        boolean prevRb = false, prevLb = false;
        // OFF by default — standard AprilTag pipelines don't need orientation.
        // Enable with RB only if testing a MegaTag2 pipeline.
        boolean sendOrientation = false;

        while (opModeIsActive()) {

            // Pipeline switching (rising-edge)
            boolean curA  = gamepad1.a;
            boolean curB  = gamepad1.b;
            boolean curX  = gamepad1.x;
            boolean curY  = gamepad1.y;
            boolean curRb = gamepad1.right_bumper;
            boolean curLb = gamepad1.left_bumper;

            if (curA  && !prevA)  { currentPipeline = 0; ll.pipelineSwitch(0); }
            if (curB  && !prevB)  { currentPipeline = 1; ll.pipelineSwitch(1); }
            if (curX  && !prevX)  { currentPipeline = 2; ll.pipelineSwitch(2); }
            if (curY  && !prevY)  { currentPipeline = 3; ll.pipelineSwitch(3); }
            if (curRb && !prevRb) { sendOrientation = !sendOrientation; }
            if (curLb && !prevLb) { totalFrames = 0; validFrames = 0; nonNullFrames = 0; }

            prevA = curA; prevB = curB; prevX = curX; prevY = curY;
            prevRb = curRb; prevLb = curLb;

            // MegaTag2 orientation feed (robot stationary = 0°)
            if (sendOrientation) ll.updateRobotOrientation(0);

            LLResult result = ll.getLatestResult();
            totalFrames++;

            // ── Status (FPS=0 means Limelight is not processing frames) ──
            LLStatus status = ll.getStatus();
            int fps = (int) status.getFps();
            String pipeType = status.getPipelineType();

            // ── Telemetry ─────────────────────────────────────────────────
            telemetry.addData("=== LINK ===", "");
            telemetry.addData("HW name",    HardwareConfig.LIMELIGHT_NAME);
            telemetry.addData("Pipeline",   currentPipeline + "  (A/B/X/Y to switch)");
            telemetry.addData("PipeType",   pipeType + "  (should be 'fiducial' for AprilTag)");
            telemetry.addData("FPS",        fps + (fps == 0 ? "  !! NO FRAMES — check USB cable !!" : ""));
            telemetry.addData("CPU / Temp", String.format("%.0f%%  /  %.1f°C", status.getCpu(), status.getTemp()));
            telemetry.addData("Orientation", sendOrientation
                    ? "SENDING updateRobotOrientation(0)  [RB to disable]"
                    : "NOT sending (standard AprilTag)    [RB to enable for MegaTag2]");

            telemetry.addData("=== FRAME COUNTS ===", "");
            telemetry.addData("Total polled", totalFrames);
            telemetry.addData("Non-null",     nonNullFrames);
            telemetry.addData("isValid",      validFrames);

            if (result == null) {
                telemetry.addData("=== RESULT ===", "NULL");
                telemetry.addLine("  -> Limelight not sending frames.");
                telemetry.addLine("  -> Check USB cable + Control Hub port.");
                telemetry.addLine("  -> Check Limelight web UI is reachable (192.168.43.1:5801).");
            } else {
                nonNullFrames++;
                if (result.isValid()) validFrames++;

                telemetry.addData("=== RESULT ===", result.isValid() ? "VALID" : "stale (isValid=false)");
                telemetry.addData("TX",  String.format("%.2f°", result.getTx()));
                telemetry.addData("TY",  String.format("%.2f°", result.getTy()));
                telemetry.addData("TA",  String.format("%.3f%%", result.getTa()));

                // AprilTag / fiducial results
                List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                int fidCount = (fids != null) ? fids.size() : -1;
                telemetry.addData("Fiducials",  fidCount < 0 ? "null list" : fidCount + " tag(s)");
                if (fids != null) {
                    for (LLResultTypes.FiducialResult f : fids) {
                        telemetry.addData("  Tag",
                                "id=" + f.getFiducialId()
                                + "  tx=" + String.format("%.1f°", f.getTargetXDegrees())
                                + "  ty=" + String.format("%.1f°", f.getTargetYDegrees()));
                    }
                }

                if (fidCount == 0) {
                    telemetry.addLine("  -> No tags seen. Check:");
                    telemetry.addLine("       1) Is pipeline " + currentPipeline + " an AprilTag pipeline?");
                    telemetry.addLine("       2) Is the tag in field of view?");
                    telemetry.addLine("       3) Is ambient light sufficient?");
                }

                // Color results
                List<LLResultTypes.ColorResult> colors = result.getColorResults();
                int colorCount = (colors != null) ? colors.size() : -1;
                telemetry.addData("Color blobs", colorCount < 0 ? "null list" : colorCount + " blob(s)");

                if (!result.isValid() && nonNullFrames > 10) {
                    telemetry.addLine("  -> isValid() stuck false: result arrives but is");
                    telemetry.addLine("     marked stale. Limelight may be outputting at");
                    telemetry.addLine("     < robot-loop rate — this is OK, ignore isValid().");
                }
            }

            telemetry.addData("LB", "reset counters");
            telemetry.update();
        }

        ll.stop();
    }
}
