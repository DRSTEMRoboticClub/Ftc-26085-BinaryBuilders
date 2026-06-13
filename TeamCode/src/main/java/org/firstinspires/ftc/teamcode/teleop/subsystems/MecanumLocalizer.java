package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;
import org.firstinspires.ftc.teamcode.configs.LocalizationConfig;

/**
 * Road Runner 1.0 mecanum dead-reckoning localizer.
 *
 * Wheel encoders (FL/FR/BL/BR) supply translation; the IMU supplies heading.
 * Each call to {@link #update()} folds the latest motion into the running
 * {@link Pose2d} estimate using Road Runner's SE(2) pose exponential
 * ({@code pose.plus(Twist2d)}), exactly as the RR localization framework does.
 *
 * Coordinate frame: +X forward, +Y to robot-left, heading CCW-positive.
 */
public class MecanumLocalizer {

    private final OverflowEncoder fl, fr, bl, br;
    private final IMU imu;

    private Pose2d pose = new Pose2d(0, 0, 0);

    // Previous raw encoder ticks and heading, for delta computation.
    private int lastFl, lastFr, lastBl, lastBr;
    private double lastHeadingRad;
    private boolean initialized = false;

    public MecanumLocalizer(HardwareMap hMap, IMU imu) {
        // Wrap each drive motor's encoder. OverflowEncoder transparently handles
        // the REV hub's 16-bit velocity rollover so high-speed counts stay valid.
        fl = new OverflowEncoder(new RawEncoder(hMap.get(DcMotorEx.class, HardwareConfig.FL_NAME)));
        fr = new OverflowEncoder(new RawEncoder(hMap.get(DcMotorEx.class, HardwareConfig.FR_NAME)));
        bl = new OverflowEncoder(new RawEncoder(hMap.get(DcMotorEx.class, HardwareConfig.BL_NAME)));
        br = new OverflowEncoder(new RawEncoder(hMap.get(DcMotorEx.class, HardwareConfig.BR_NAME)));
        this.imu = imu;
    }

    /** Force the estimate to a known pose (used by AprilTag corrections / reset). */
    public void setPose(Pose2d newPose) {
        this.pose = newPose;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getXInches() {
        return pose.position.x;
    }

    public double getYInches() {
        return pose.position.y;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(pose.heading.toDouble());
    }

    /** Read the IMU yaw (radians, CCW-positive). */
    private double readImuHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Update the pose estimate from the latest encoder + IMU readings.
     * Call once per control loop, after clearing the hub bulk cache.
     */
    public void update() {
        PositionVelocityPair pFl = fl.getPositionAndVelocity();
        PositionVelocityPair pFr = fr.getPositionAndVelocity();
        PositionVelocityPair pBl = bl.getPositionAndVelocity();
        PositionVelocityPair pBr = br.getPositionAndVelocity();
        double headingRad = readImuHeadingRad();

        if (!initialized) {
            // First pass: seed baselines, emit no motion.
            lastFl = pFl.position;
            lastFr = pFr.position;
            lastBl = pBl.position;
            lastBr = pBr.position;
            lastHeadingRad = headingRad;
            initialized = true;
            return;
        }

        // Per-wheel travel in inches since last loop (with calibration signs).
        double dFl = (pFl.position - lastFl) * LocalizationConfig.WHEEL_IN_PER_TICK * LocalizationConfig.FL_TICK_SIGN;
        double dFr = (pFr.position - lastFr) * LocalizationConfig.WHEEL_IN_PER_TICK * LocalizationConfig.FR_TICK_SIGN;
        double dBl = (pBl.position - lastBl) * LocalizationConfig.WHEEL_IN_PER_TICK * LocalizationConfig.BL_TICK_SIGN;
        double dBr = (pBr.position - lastBr) * LocalizationConfig.WHEEL_IN_PER_TICK * LocalizationConfig.BR_TICK_SIGN;

        lastFl = pFl.position;
        lastFr = pFr.position;
        lastBl = pBl.position;
        lastBr = pBr.position;

        // Mecanum forward kinematics -> robot-relative displacement this loop.
        //   forward  = average of all four wheels
        //   left     = (-FL + FR + BL - BR)/4, scaled by lateral slip multiplier
        double dForward = (dFl + dFr + dBl + dBr) / 4.0;
        double dLeft = ((-dFl + dFr + dBl - dBr) / 4.0) * LocalizationConfig.LATERAL_MULTIPLIER;

        // Heading change comes from the IMU, not the wheels (more accurate).
        double dHeading = normalize(headingRad - lastHeadingRad);
        lastHeadingRad = headingRad;

        // Fold the robot-relative twist into the field pose via RR's SE(2)
        // exponential. This arc-integrates translation across the heading change.
        Twist2d twist = new Twist2d(new Vector2d(dForward, dLeft), dHeading);
        pose = pose.plus(twist);

        // Pin heading to the absolute IMU reading to stop integrator drift.
        pose = new Pose2d(pose.position, headingRad);
    }

    private static double normalize(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad <= -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
