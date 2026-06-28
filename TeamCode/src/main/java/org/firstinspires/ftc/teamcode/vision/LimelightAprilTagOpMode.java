package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;

import java.io.File;
import java.util.List;

/**
 * Limelight 3A AprilTag-only reader for FTC SDK 11.x.
 *
 * On init it:
 *   1. grabs the Limelight3A from the hardware map,
 *   2. loads an AprilTag pipeline JSON from  /sdcard/FIRST/limelight/apriltag.json,
 *   3. uploads it into pipeline slot 0 ({@code uploadPipeline}),
 *   4. selects slot 0 ({@code pipelineSwitch}),
 *   5. starts the camera.
 *
 * Re-uploading the same known-good JSON from disk every init is the reliable way to stop the
 * Limelight from drifting / losing its configuration between power cycles — provided the file
 * on disk is a COMPLETE pipeline exported from this Limelight's own web UI (see the JSON notes
 * in the chat / repo docs). A hand-truncated JSON can corrupt the pipeline, so always seed the
 * file from a real export.
 *
 * Every loop it reads the latest result and reports 2D detections only — tag id, tx, ty, ta,
 * latency and pipeline index. 3D pose fields are intentionally never queried.
 *
 * All failure modes (no device, missing file, failed upload, failed switch) are surfaced on
 * telemetry instead of crashing the OpMode.
 */
@TeleOp(name = "Limelight AprilTag (JSON upload)", group = "Vision")
public class LimelightAprilTagOpMode extends LinearOpMode {

    /** Pipeline slot we upload to and run. */
    private static final int PIPELINE_INDEX = 0;

    /** Poll the device faster than the 40 fps pipeline so we always read the freshest frame. */
    private static final int POLL_RATE_HZ = 100;

    /** /sdcard/FIRST/limelight/apriltag.json  (FIRST_FOLDER == /sdcard/FIRST). */
    private static final File PIPELINE_FILE =
            new File(AppUtil.FIRST_FOLDER, "limelight/apriltag.json");

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        StringBuilder errors = new StringBuilder();

        // ── 1. Acquire the device ────────────────────────────────────────────────
        try {
            limelight = hardwareMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT_NAME);
        } catch (Exception e) {
            limelight = null;
            errors.append("No Limelight named '")
                  .append(HardwareConfig.LIMELIGHT_NAME)
                  .append("' in the robot configuration. ");
        }

        // ── 2-4. Load + upload + select the pipeline ─────────────────────────────
        boolean ready = (limelight != null) && configurePipeline(errors);

        // ── 5. Start polling ─────────────────────────────────────────────────────
        if (ready) {
            limelight.setPollRateHz(POLL_RATE_HZ);
            limelight.start();
        }

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine(ready ? "Limelight ready — press PLAY." : "INIT FAILED — see below:");
        if (errors.length() > 0) telemetry.addLine(errors.toString());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (!ready) {
                // Init failed: keep reporting why, don't pretend to track.
                telemetry.addLine("Limelight NOT running.");
                telemetry.addLine(errors.toString());
                telemetry.update();
                sleep(250);
                continue;
            }

            // Runtime disconnect (USB pulled / power) — report and keep going.
            if (!limelight.isConnected()) {
                telemetry.addLine("Limelight DISCONNECTED (USB / power?).");
                telemetry.update();
                sleep(100);
                continue;
            }

            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

            telemetry.addData("Health", "fps=%.0f cpu=%.0f%% temp=%.0fC",
                    status.getFps(), status.getCpu(), status.getTemp());
            telemetry.addData("Pipeline", "idx=%d type=%s", status.getPipelineIndex(),
                    status.getPipelineType());

            if (result == null) {
                telemetry.addLine("No result yet (null).");
            } else if (!result.isValid()) {
                telemetry.addData("Result", "invalid (staleness=%d ms)", result.getStaleness());
            } else {
                // Latency = time to capture the frame + time the pipeline took to process it.
                double latencyMs = result.getCaptureLatency() + result.getTargetingLatency();

                telemetry.addData("Latency", "%.1f ms  (capture %.1f + pipeline %.1f)",
                        latencyMs, result.getCaptureLatency(), result.getTargetingLatency());
                telemetry.addData("Pipeline index (from result)", result.getPipelineIndex());
                telemetry.addData("Primary target", "tx=%.2f  ty=%.2f  ta=%.3f",
                        result.getTx(), result.getTy(), result.getTa());

                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                telemetry.addData("AprilTags seen", tags == null ? 0 : tags.size());
                if (tags != null) {
                    for (LLResultTypes.FiducialResult tag : tags) {
                        // 2D detection fields only — 3D pose getters are intentionally ignored.
                        telemetry.addData("  Tag " + tag.getFiducialId(),
                                "fam=%s  tx=%.2f  ty=%.2f  ta=%.3f",
                                tag.getFamily(),
                                tag.getTargetXDegrees(),
                                tag.getTargetYDegrees(),
                                tag.getTargetArea());
                    }
                }
            }
            telemetry.update();
        }

        if (limelight != null) {
            limelight.stop();
        }
    }

    /**
     * Reads the pipeline JSON from disk and pushes it to the Limelight, then selects it.
     * Returns true only if the whole chain succeeded; otherwise appends a human-readable
     * reason to {@code errors} and returns false.
     */
    private boolean configurePipeline(StringBuilder errors) {
        // Missing file.
        if (!PIPELINE_FILE.exists()) {
            errors.append("Pipeline file not found at ")
                  .append(PIPELINE_FILE.getAbsolutePath())
                  .append(". Export an AprilTag pipeline from the Limelight UI and copy it there. ");
            return false;
        }

        // Unreadable / empty file (ReadWriteFile.readFile returns "" on IO failure).
        String json = ReadWriteFile.readFile(PIPELINE_FILE);
        if (json == null || json.trim().isEmpty()) {
            errors.append("Pipeline file is empty or unreadable. ");
            return false;
        }

        // Upload (returns false on a bad JSON / bad index / comms failure).
        boolean uploaded;
        try {
            uploaded = limelight.uploadPipeline(json, PIPELINE_INDEX);
        } catch (Exception e) {
            errors.append("uploadPipeline() threw: ").append(e.getMessage()).append(". ");
            return false;
        }
        if (!uploaded) {
            errors.append("uploadPipeline() returned false (malformed JSON or bad index ")
                  .append(PIPELINE_INDEX).append("?). ");
            return false;
        }

        // Select the freshly uploaded pipeline.
        boolean switched;
        try {
            switched = limelight.pipelineSwitch(PIPELINE_INDEX);
        } catch (Exception e) {
            errors.append("pipelineSwitch() threw: ").append(e.getMessage()).append(". ");
            return false;
        }
        if (!switched) {
            errors.append("pipelineSwitch(").append(PIPELINE_INDEX).append(") returned false. ");
            return false;
        }
        return true;
    }
}
