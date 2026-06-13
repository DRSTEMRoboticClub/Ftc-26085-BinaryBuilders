package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.configs.LocalizationConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.teleop.localization.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.teleop.localization.TurretTracker;
import org.firstinspires.ftc.teamcode.teleop.subsystems.*;
import org.firstinspires.ftc.teamcode.tools.PriorityInputHandler;

import java.util.List;

/**
 * Shared base for the LocalSys localization-testing TeleOps.
 *
 * Behaves EXACTLY like {@link TeleOpBlue} for driving / shooter / intake / hood
 * (same subsystems, same {@link PriorityInputHandler}), and layers on:
 *   - Road Runner mecanum dead reckoning ({@link MecanumLocalizer})
 *   - turret-encoder based turret tracking + cable-safe wraparound ({@link TurretTracker})
 *   - turret-compensated AprilTag pose correction ({@link AprilTagLocalizer})
 *   - full localization telemetry
 *
 * Blue / Red differ only in the alliance constants returned by the abstract
 * getters below, so there is no duplicated logic between the two OpModes.
 */
public abstract class LocalSysBase extends CommandOpMode {

    // ---- Alliance-specific values supplied by the Blue / Red subclasses ----
    protected abstract int getTagId();
    protected abstract double getTagFieldX();
    protected abstract double getTagFieldY();
    protected abstract String getAllianceName();

    // ---- Standard TeleOpBlue subsystems / IO ----
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private HoodSubsystem hood;
    private PriorityInputHandler inputHandler;
    private GamepadEx g1, g2;
    private VoltageSensor batteryVoltageSensor;
    private List<LynxModule> allHubs;

    private long lastLoopTime = 0;
    private double minVoltage = 14.0;

    // ---- Localization ----
    private MecanumLocalizer localizer;
    private TurretTracker turretTracker;

    // Diagnostics snapshots for telemetry.
    private Pose2d lastCorrectedPose = null;
    private double poseErrorIn = 0;
    private AprilTagLocalizer.Observation lastObs = new AprilTagLocalizer.Observation();

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        hood = new HoodSubsystem(hardwareMap);

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        inputHandler = new PriorityInputHandler(g1, g2);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.visuallyIdentify(false);
        }

        // Localization shares the drive's already-initialized IMU.
        localizer = new MecanumLocalizer(hardwareMap, drive.getImu());
        turretTracker = new TurretTracker(shooter);

        // Make sure this OpMode tracks ITS alliance tag everywhere it matters.
        ShooterConfig.TRACKED_TAG_ID = getTagId();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            long currentTime = System.currentTimeMillis();
            long loopTime = (lastLoopTime == 0) ? 0 : currentTime - lastLoopTime;
            lastLoopTime = currentTime;

            // 1) Standard TeleOpBlue control (unchanged behaviour).
            inputHandler.update(drive, intake, shooter, hood);

            // 2) Localization subsystems (each kept in its own helper).
            localizer.update();
            turretTracker.update(shooter, getTagId());   // overrides turret for auto-track
            applyAprilTagCorrection();

            // 3) Telemetry.
            double voltage = batteryVoltageSensor.getVoltage();
            if (voltage < minVoltage) minVoltage = voltage;
            renderTelemetry(voltage, loopTime);
            telemetry.update();
        }

        shooter.stopLimelight();
    }

    /**
     * Fuse an AprilTag observation into the Road Runner pose estimate.
     * The turret angle is fed in so the measurement is corrected for turret
     * rotation before it touches the pose (see {@link AprilTagLocalizer}).
     */
    private void applyAprilTagCorrection() {
        double turretRad = Math.toRadians(turretTracker.getLastTurretAngleDegrees());
        double headingRad = Math.toRadians(localizer.getHeadingDegrees());

        AprilTagLocalizer.Observation obs = AprilTagLocalizer.correct(
                shooter.getLimelightResult(), getTagId(),
                turretRad, headingRad,
                getTagFieldX(), getTagFieldY());
        lastObs = obs;

        if (obs.correctedPose == null) return; // no trusted tag this frame

        Pose2d rr = localizer.getPose();
        lastCorrectedPose = obs.correctedPose;
        poseErrorIn = Math.hypot(
                rr.position.x - obs.correctedPose.position.x,
                rr.position.y - obs.correctedPose.position.y);

        // Low-pass blend RR toward the tag estimate to reject jitter.
        double a = LocalizationConfig.TAG_CORRECTION_ALPHA;
        double fusedX = rr.position.x + a * (obs.correctedPose.position.x - rr.position.x);
        double fusedY = rr.position.y + a * (obs.correctedPose.position.y - rr.position.y);
        // Keep heading from the IMU-backed RR estimate.
        localizer.setPose(new Pose2d(fusedX, fusedY, rr.heading.toDouble()));
    }

    private void renderTelemetry(double voltage, long loopTime) {
        telemetry.addData("Alliance", "%s  (tracking tag %d, 36h11)", getAllianceName(), getTagId());

        telemetry.addLine("=== ROAD RUNNER POSE ===");
        telemetry.addData("X", "%.1f", localizer.getXInches());
        telemetry.addData("Y", "%.1f", localizer.getYInches());
        telemetry.addData("Heading", "%.1f°", localizer.getHeadingDegrees());

        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Encoder", shooter.getTurretTicks());
        telemetry.addData("Angle", "%.1f°", turretTracker.getLastTurretAngleDegrees());
        telemetry.addData("State", turretTracker.isUnwinding() ? "UNWINDING (cable limit)" : "TRACKING");

        telemetry.addLine("=== APRILTAG ===");
        telemetry.addData("Detected ID", lastObs.tagId);
        telemetry.addData("Visible", lastObs.visible);
        telemetry.addData("Distance", "%.1f in", lastObs.distanceIn);
        telemetry.addData("Yaw (tx)", "%.1f°", lastObs.txDeg);
        telemetry.addData("Bearing (robot)", "%.1f°", lastObs.bearingRobotDeg);

        telemetry.addLine("=== LOCALIZATION DIAGNOSTICS ===");
        telemetry.addData("RR Pose", "(%.1f, %.1f, %.1f°)",
                localizer.getXInches(), localizer.getYInches(), localizer.getHeadingDegrees());
        if (lastCorrectedPose != null) {
            telemetry.addData("Tag Corrected Pose", "(%.1f, %.1f, %.1f°)",
                    lastCorrectedPose.position.x, lastCorrectedPose.position.y,
                    Math.toDegrees(lastCorrectedPose.heading.toDouble()));
        } else {
            telemetry.addData("Tag Corrected Pose", "-- no fix yet --");
        }
        telemetry.addData("Pose Error", "%.1f in", poseErrorIn);

        telemetry.addLine("=== HEALTH ===");
        String voltStatus = (voltage < 11.0) ? "!! BROWNOUT RISK !!" : (voltage < 12.0) ? "! LOW !" : "OK";
        String loopStatus = (loopTime > 45) ? "!! HIGH LATENCY !!" : "OK";
        telemetry.addData("Battery", "%.2fV [%s]", voltage, voltStatus);
        telemetry.addData("Loop Time", "%d ms [%s]", loopTime, loopStatus);
    }
}
