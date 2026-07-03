package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;

import java.util.Collections;
import java.util.List;

/**
 * Central manager for all Limelight3A operations.
 *
 * Consolidates functions previously scattered across ShooterSubsystem,
 * AutoFSMBlue, and LimelightHealthCheck into one place:
 *   - Lifecycle (start / stop / pipeline switching)
 *   - Status diagnostics (FPS, pipeline type, connection)
 *   - AprilTag queries (find by ID, TX/TY, 3D camera-space pose, distance)
 *   - Color blob queries (all blobs, largest blob, TX / area)
 *   - MegaTag2 orientation feed
 *
 * Usage:
 *   LimelightManager ll = new LimelightManager(hardwareMap);
 *   ll.switchToAprilTag();           // pipeline 0
 *   ll.feedHeading(drive.getHeading()); // every loop (MegaTag2 only)
 *   if (ll.seesTag(20)) { double tx = ll.getTagTx(20); }
 */
public class LimelightManager {

    // ── Standard pipeline indices ──────────────────────────────────────────
    public static final int PIPELINE_APRILTAG = ShooterConfig.APRILTAG_PIPELINE; // 0
    public static final int PIPELINE_COLOR     = 1;  // colour-blob ball detection

    private final Limelight3A limelight;
    private int currentPipeline = PIPELINE_APRILTAG;

    // ── Construction ──────────────────────────────────────────────────────

    /**
     * Get the Limelight from the hardware map and start it on the AprilTag pipeline.
     */
    public LimelightManager(HardwareMap hMap) {
        this(hMap, PIPELINE_APRILTAG);
    }

    /**
     * Get the Limelight from the hardware map and start it on {@code startPipeline}.
     */
    public LimelightManager(HardwareMap hMap, int startPipeline) {
        limelight = hMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT_NAME);
        limelight.pipelineSwitch(startPipeline);
        limelight.start();
        currentPipeline = startPipeline;
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────

    /** Stop streaming. Call at the end of the OpMode. */
    public void stop() {
        limelight.stop();
    }

    /** Switch to an arbitrary pipeline index. */
    public void switchPipeline(int index) {
        if (index != currentPipeline) {
            limelight.pipelineSwitch(index);
            currentPipeline = index;
        }
    }

    /** Switch to the AprilTag pipeline (pipeline 0 by default). */
    public void switchToAprilTag() {
        switchPipeline(PIPELINE_APRILTAG);
    }

    /** Switch to the colour-blob ball-detection pipeline (pipeline 1 by default). */
    public void switchToColor() {
        switchPipeline(PIPELINE_COLOR);
    }

    /**
     * Feed the robot's current IMU yaw to the Limelight.
     * Required every loop when using a MegaTag2 pipeline; harmless for standard AprilTag.
     *
     * @param yawDeg robot heading in degrees (from IMU, e.g. DriveSubsystem.getHeading())
     */
    public void feedHeading(double yawDeg) {
        limelight.updateRobotOrientation(yawDeg);
    }

    public int getCurrentPipeline() { return currentPipeline; }

    // ── Status / diagnostics ──────────────────────────────────────────────

    /** FPS reported by the Limelight. 0 means no frames are being processed. */
    public int getFps() {
        return (int) limelight.getStatus().getFps();
    }

    /**
     * Pipeline type string reported by the Limelight (e.g. "fiducial", "color").
     * Useful to confirm that the correct pipeline is actually active.
     */
    public String getPipelineType() {
        return limelight.getStatus().getPipelineType();
    }

    /** CPU load percentage (0–100). */
    public double getCpuPercent() {
        return limelight.getStatus().getCpu();
    }

    /** Limelight board temperature in Celsius. */
    public double getTemperatureC() {
        return limelight.getStatus().getTemp();
    }

    /**
     * One-line summary for driver-station telemetry.
     * Example: "fps=30 pipe=fiducial valid=true fids=1 tx=-5.2°"
     */
    public String getDebugInfo() {
        LLStatus s  = limelight.getStatus();
        int fps     = (int) s.getFps();
        String pipe = s.getPipelineType();
        LLResult r  = limelight.getLatestResult();
        if (r == null) return String.format("fps=%d pipe=%s result=NULL", fps, pipe);
        int fids = (r.getFiducialResults() != null) ? r.getFiducialResults().size() : -1;
        return String.format("fps=%d pipe=%s valid=%b fids=%d tx=%.1f°",
                fps, pipe, r.isValid(), fids, r.getTx());
    }

    /**
     * Full status block suitable for the LL Health Check display.
     * Appends a warning when FPS=0.
     */
    public String getStatusSummary() {
        LLStatus s = limelight.getStatus();
        int fps    = (int) s.getFps();
        return String.format("fps=%d%s  pipe=%s  cpu=%.0f%%  temp=%.1f°C",
                fps,
                fps == 0 ? " !! NO FRAMES" : "",
                s.getPipelineType(),
                s.getCpu(),
                s.getTemp());
    }

    // ── Raw result access ─────────────────────────────────────────────────

    /**
     * Latest Limelight result. May be null if start() was never called or the
     * Limelight has not produced any frames yet. Prefer the typed helpers below.
     */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * True if the Limelight has produced a fresh frame since the last poll.
     * False does NOT mean "no data" — it means the last frame is being re-used.
     */
    public boolean hasValidResult() {
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid();
    }

    // ── AprilTag helpers ──────────────────────────────────────────────────

    /** All fiducial (AprilTag) results in the current frame, or an empty list. */
    public List<LLResultTypes.FiducialResult> getAllTags() {
        LLResult r = limelight.getLatestResult();
        if (r == null) return Collections.emptyList();
        List<LLResultTypes.FiducialResult> fids = r.getFiducialResults();
        return (fids != null) ? fids : Collections.emptyList();
    }

    /** True if the Limelight currently sees a tag with the given ID. */
    public boolean seesTag(int tagId) {
        return findTag(tagId) != null;
    }

    /**
     * Horizontal offset (degrees) of the tag from the camera centre.
     * Positive = tag is to the right. Returns null if the tag is not visible.
     */
    public Double getTagTx(int tagId) {
        LLResultTypes.FiducialResult f = findTag(tagId);
        return (f != null) ? f.getTargetXDegrees() : null;
    }

    /**
     * Vertical offset (degrees) of the tag from the camera centre.
     * Positive = tag is above centre. Returns null if not visible.
     */
    public Double getTagTy(int tagId) {
        LLResultTypes.FiducialResult f = findTag(tagId);
        return (f != null) ? f.getTargetYDegrees() : null;
    }

    /**
     * 3D pose of the tag in camera space (Limelight's native output).
     * Axes: +X right, +Y up, +Z forward (into the scene).
     * Returns null if the tag is not visible or 3D pose is unavailable.
     */
    public Pose3D getTagCamSpacePose(int tagId) {
        LLResultTypes.FiducialResult f = findTag(tagId);
        return (f != null) ? f.getTargetPoseCameraSpace() : null;
    }

    /**
     * Planar (floor-plane) distance from the camera to the tag in centimetres.
     * Projects camera-space coordinates onto the horizontal plane, accounting for
     * the camera's upward mount tilt (ShooterConfig.CAMERA_TILT_DEG).
     * Returns -1 if the tag is not visible or pose data is unavailable.
     */
    public double getTagDistanceCm(int tagId) {
        Pose3D cam = getTagCamSpacePose(tagId);
        if (cam == null) return -1;
        var pos = cam.getPosition();
        if (pos == null) return -1;  // 3D pose unavailable for this tag
        double xRight = pos.toUnit(DistanceUnit.CM).x;
        double yUp    = pos.toUnit(DistanceUnit.CM).y;
        double zFwd   = pos.toUnit(DistanceUnit.CM).z;
        double tilt   = Math.toRadians(ShooterConfig.CAMERA_TILT_DEG);
        double hFwd   = zFwd * Math.cos(tilt) - yUp * Math.sin(tilt);
        return Math.hypot(xRight, hFwd);
    }

    /**
     * Comma-separated list of all visible AprilTag IDs. Returns "none" if none seen.
     * Useful for driver-station telemetry.
     */
    public String getVisibleTagIds() {
        List<LLResultTypes.FiducialResult> fids = getAllTags();
        if (fids.isEmpty()) return "none";
        StringBuilder sb = new StringBuilder();
        for (LLResultTypes.FiducialResult f : fids) {
            if (sb.length() > 0) sb.append(", ");
            sb.append(f.getFiducialId());
        }
        return sb.toString();
    }

    // ── Color-blob helpers ────────────────────────────────────────────────

    /** All colour blob results in the current frame, or an empty list. */
    public List<LLResultTypes.ColorResult> getColorBlobs() {
        LLResult r = limelight.getLatestResult();
        if (r == null) return Collections.emptyList();
        List<LLResultTypes.ColorResult> blobs = r.getColorResults();
        return (blobs != null) ? blobs : Collections.emptyList();
    }

    /**
     * The single largest colour blob by area — the closest / most prominent ball.
     * Returns null if no blobs are visible.
     */
    public LLResultTypes.ColorResult getLargestBlob() {
        List<LLResultTypes.ColorResult> blobs = getColorBlobs();
        if (blobs.isEmpty()) return null;
        LLResultTypes.ColorResult best = blobs.get(0);
        for (LLResultTypes.ColorResult b : blobs) {
            if (b.getTargetArea() > best.getTargetArea()) best = b;
        }
        return best;
    }

    /**
     * Horizontal offset (degrees) of the largest colour blob.
     * Returns null if no blobs are visible.
     */
    public Double getBlobTx() {
        LLResultTypes.ColorResult blob = getLargestBlob();
        return (blob != null) ? blob.getTargetXDegrees() : null;
    }

    /**
     * Area percentage (0–100) of the largest colour blob.
     * Larger area = ball is closer to the camera / intake.
     * Returns 0 if no blobs are visible.
     */
    public double getBlobArea() {
        LLResultTypes.ColorResult blob = getLargestBlob();
        return (blob != null) ? blob.getTargetArea() : 0.0;
    }

    // ── Internal helpers ──────────────────────────────────────────────────

    private LLResultTypes.FiducialResult findTag(int tagId) {
        for (LLResultTypes.FiducialResult f : getAllTags()) {
            if (f.getFiducialId() == tagId) return f;
        }
        return null;
    }
}
