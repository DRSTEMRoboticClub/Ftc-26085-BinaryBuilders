package org.firstinspires.ftc.teamcode.autoop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.tools.localization.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.tools.localization.TurretTracker;
import org.firstinspires.ftc.teamcode.teleop.subsystems.ShooterSubsystem;

/**
 * Drives a PedroPathing {@link Follower} for autonomous, and OPTIONALLY layers the
 * team's existing turret-compensated AprilTag correction on top of Pedro's odometry.
 *
 * Two ways to use it:
 *   - Plain path following: just {@code new PedroAutoRunner(hardwareMap)}, set a
 *     start pose, follow paths, and call {@link #update()} every loop. Localization
 *     comes entirely from Pedro's drive-encoder odometry. This works out of the box.
 *
 *   - With AprilTag correction: call {@link #enableAprilTagCorrection}. Every loop
 *     the turret is kept pointed at the alliance tag (reusing {@link TurretTracker})
 *     and a field-pose fix is computed (reusing {@link AprilTagLocalizer}) and
 *     blended into Pedro's pose. This is the SAME localization stack the
 *     {@code LocalSys} TeleOps use, so behaviour matches between teleop and auto.
 *
 * COORDINATE-FRAME WARNING for AprilTag correction:
 *   {@link AprilTagLocalizer} reports the robot pose in the field frame implied by
 *   the tag field coordinates you pass in ({@code LocalizationConfig.*_TAG_FIELD_*},
 *   a center-origin convention). Pedro's pose lives in whatever frame you gave
 *   {@link #setStartPose}. For the blended correction to be meaningful, those two
 *   frames must match. Get plain path following + localization solid FIRST, then
 *   turn on tag correction and verify the corrected pose lines up before trusting it.
 */
@Config
public class PedroAutoRunner {

    /** Master switch for fusing AprilTag fixes into Pedro's pose. */
    public static boolean USE_APRILTAG_CORRECTION = false;
    /** Blend factor each frame: 0 = ignore tag, 1 = snap fully to tag. */
    public static double TAG_BLEND_ALPHA = 0.20;

    private final Follower follower;

    // --- Optional AprilTag correction stack (only built if enabled) ---
    private ShooterSubsystem shooter;
    private TurretTracker turretTracker;
    private int tagId;
    private double tagFieldX, tagFieldY;
    private boolean tagStackReady = false;

    private AprilTagLocalizer.Observation lastObs = new AprilTagLocalizer.Observation();
    private Pose lastCorrectedPose = null;

    public PedroAutoRunner(HardwareMap hMap) {
        follower = PedroConstants.createFollower(hMap);
    }

    /** Direct access to the Follower for path building, holdPoint(), etc. */
    public Follower getFollower() {
        return follower;
    }

    /** Set the robot's known starting field pose. Call before following anything. */
    public void setStartPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    /**
     * Turn on turret-compensated AprilTag corrections, reusing the team's existing
     * localization classes.
     *
     * @param shooter    owns the Limelight + turret encoder (already on the robot)
     * @param tagId      alliance AprilTag to track and localize from
     * @param tagFieldX  that tag's known field X (inches)
     * @param tagFieldY  that tag's known field Y (inches)
     */
    public void enableAprilTagCorrection(ShooterSubsystem shooter, int tagId,
                                         double tagFieldX, double tagFieldY) {
        this.shooter = shooter;
        this.turretTracker = new TurretTracker(shooter);
        this.tagId = tagId;
        this.tagFieldX = tagFieldX;
        this.tagFieldY = tagFieldY;
        this.tagStackReady = true;
        ShooterConfig.TRACKED_TAG_ID = tagId;
        USE_APRILTAG_CORRECTION = true;
        shooter.setRetainCachedResult(true); // AprilTagLocalizer needs the raw LLResult
    }

    public void followPath(Path path) {
        follower.followPath(path, true);
    }

    public void followPath(PathChain chain) {
        follower.followPath(chain, true);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    /**
     * Advance the follower one control loop, and fuse an AprilTag fix if enabled.
     * Call exactly once per loop iteration.
     */
    public void update() {
        follower.update();
        if (USE_APRILTAG_CORRECTION && tagStackReady) {
            // Turret tracking uses the cached TX primitive — safe to run every loop.
            turretTracker.update(shooter, shooter.getTrackedTagTx());
            // 3D pose solver in AprilTagLocalizer is expensive; only run when the
            // LL cache has fresh data (same 250ms cadence as cacheLimelightResult).
            if (shooter.wasResultUpdated()) {
                applyAprilTagCorrection();
            }
        }
    }

    private void applyAprilTagCorrection() {
        double turretRad = Math.toRadians(turretTracker.getLastTurretAngleDegrees());

        Pose current = follower.getPose();
        double headingRad = current.getHeading();

        AprilTagLocalizer.Observation obs = AprilTagLocalizer.correct(
                shooter.getLimelightResult(), tagId,
                turretRad, headingRad, tagFieldX, tagFieldY);
        lastObs = obs;
        if (obs.correctedPose == null) return; // no trusted tag this frame

        lastCorrectedPose = obs.correctedPose;
        double a = TAG_BLEND_ALPHA;
        double fusedX = current.getX() + a * (lastCorrectedPose.getX() - current.getX());
        double fusedY = current.getY() + a * (lastCorrectedPose.getY() - current.getY());
        // Heading stays from Pedro odometry; only X/Y are nudged toward the tag.
        follower.setPose(new Pose(fusedX, fusedY, current.getHeading()));
    }

    public AprilTagLocalizer.Observation getLastObservation() {
        return lastObs;
    }

    public Pose getLastCorrectedPose() {
        return lastCorrectedPose;
    }

    /** Stop following and release the Limelight (if the tag stack was used). */
    public void stop() {
        follower.breakFollowing();
        if (shooter != null) shooter.stopLimelight();
    }
}
