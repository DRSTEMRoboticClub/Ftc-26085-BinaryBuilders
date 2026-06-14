package org.firstinspires.ftc.teamcode.tools.localization;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configs.LocalizationConfig;

import java.util.List;

/**
 * Turns a Limelight AprilTag observation into a Road Runner field-pose estimate,
 * compensating for the fact that the Limelight rides on a rotating turret.
 *
 * Frames (all +X forward, +Y left, CCW+):
 *   camera frame  -> turret frame -> robot frame -> field frame
 *
 * The transform chain:
 *   1. Limelight gives the tag position in CAMERA space (right/down/forward, m).
 *   2. Convert to a planar (forward, left) vector + bearing, in inches.
 *   3. Rotate that bearing by the live TURRET angle (+ camera yaw offset) so the
 *      measurement is expressed in ROBOT coordinates. This is the key step that
 *      accounts for turret rotation.
 *   4. Add the camera's mounting position (turret pivot + camera offset rotated
 *      by the turret angle) to get the robot-center -> tag vector in robot frame.
 *   5. Rotate by the robot heading (IMU) to get that vector in FIELD frame.
 *   6. robotField = tagField - (robotCenter -> tag in field). Heading is taken
 *      straight from the IMU (a single tag bearing can't reliably give heading).
 */
public class AprilTagLocalizer {

    /** Result of attempting an AprilTag pose correction. */
    public static class Observation {
        public boolean visible = false;          // requested tag seen this frame
        public int tagId = -1;
        public double distanceIn = 0;            // planar camera->tag distance (in)
        public double txDeg = 0;                 // tag horizontal offset in camera (deg)
        public double bearingRobotDeg = 0;       // tag bearing in robot frame (deg)
        public Pose correctedPose = null;         // estimated robot field pose (null if untrusted)
    }

    /**
     * Compute a field-pose correction from the latest Limelight frame.
     *
     * @param result         latest Limelight result
     * @param tagId          alliance tag to use
     * @param turretAngleRad turret heading relative to robot (radians, CCW+)
     * @param headingRad     robot heading from IMU (radians, CCW+)
     * @param tagFieldX      known tag X on the field (inches)
     * @param tagFieldY      known tag Y on the field (inches)
     */
    public static Observation correct(LLResult result, int tagId,
                                      double turretAngleRad, double headingRad,
                                      double tagFieldX, double tagFieldY) {
        Observation obs = new Observation();
        obs.tagId = tagId;

        LLResultTypes.FiducialResult fid = findTag(result, tagId);
        if (fid == null) return obs;
        obs.visible = true;
        obs.txDeg = fid.getTargetXDegrees();

        // Step 1-2: tag position in camera space -> planar (forward, left) inches.
        Pose3D camSpace = fid.getTargetPoseCameraSpace();
        if (camSpace == null) return obs;
        double xRight = camSpace.getPosition().toUnit(DistanceUnit.INCH).x; // +right
        double zFwd = camSpace.getPosition().toUnit(DistanceUnit.INCH).z;   // +forward
        double camFwd = zFwd;
        double camLeft = -xRight; // camera +right -> robot-style +left is negative
        double distance = Math.hypot(camFwd, camLeft);
        obs.distanceIn = distance;

        // Reject implausible / far observations.
        if (distance <= 0 || distance > LocalizationConfig.TAG_MAX_TRUST_DISTANCE) {
            return obs;
        }

        // Bearing of the tag within the camera frame (left-positive).
        double bearingCam = Math.atan2(camLeft, camFwd);

        // Step 3: rotate bearing into the ROBOT frame using the turret angle.
        // The camera optical axis points along (turret angle + camera yaw offset).
        double camAxisRobot = turretAngleRad + Math.toRadians(LocalizationConfig.CAMERA_YAW_OFFSET);
        double bearingRobot = camAxisRobot + bearingCam;
        obs.bearingRobotDeg = Math.toDegrees(bearingRobot);

        // Camera->tag vector expressed in robot frame.
        double camToTagX = distance * Math.cos(bearingRobot);
        double camToTagY = distance * Math.sin(bearingRobot);

        // Step 4: camera mounting position in robot frame =
        //   turret pivot + (camera offset rotated by the turret angle).
        double cosT = Math.cos(turretAngleRad);
        double sinT = Math.sin(turretAngleRad);
        double camPosX = LocalizationConfig.TURRET_PIVOT_X
                + LocalizationConfig.CAMERA_OFFSET_X * cosT - LocalizationConfig.CAMERA_OFFSET_Y * sinT;
        double camPosY = LocalizationConfig.TURRET_PIVOT_Y
                + LocalizationConfig.CAMERA_OFFSET_X * sinT + LocalizationConfig.CAMERA_OFFSET_Y * cosT;

        // Robot-center -> tag vector, robot frame.
        double robotToTagX = camPosX + camToTagX;
        double robotToTagY = camPosY + camToTagY;

        // Step 5: rotate that vector into the field frame by the robot heading.
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double fieldVecX = robotToTagX * cosH - robotToTagY * sinH;
        double fieldVecY = robotToTagX * sinH + robotToTagY * cosH;

        // Step 6: solve for the robot's field position. Heading from IMU.
        double robotFieldX = tagFieldX - fieldVecX;
        double robotFieldY = tagFieldY - fieldVecY;
        obs.correctedPose = new Pose(robotFieldX, robotFieldY, headingRad);
        return obs;
    }

    private static LLResultTypes.FiducialResult findTag(LLResult result, int tagId) {
        if (result == null) return null;
        List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
        if (fids == null) return null;
        for (LLResultTypes.FiducialResult f : fids) {
            if (f.getFiducialId() == tagId) return f;
        }
        return null;
    }
}
