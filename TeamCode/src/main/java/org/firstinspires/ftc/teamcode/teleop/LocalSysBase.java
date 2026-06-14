package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.geometry.Pose;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.configs.LocalizationConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.teleop.subsystems.*;
import org.firstinspires.ftc.teamcode.tools.PriorityInputHandler;
import org.firstinspires.ftc.teamcode.tools.localization.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.tools.localization.MecanumLocalizer;
import org.firstinspires.ftc.teamcode.tools.localization.PanelsFieldDrawer;
import org.firstinspires.ftc.teamcode.tools.localization.TurretTracker;

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
    private Pose lastCorrectedPose = null;
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

        PanelsFieldDrawer.init();
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

            // 3) Field visualisation — draw robot position on Panels field view.
            PanelsFieldDrawer.update(localizer.getXInches(), localizer.getYInches(),
                    Math.toRadians(localizer.getHeadingDegrees()));

            // 5) Telemetry.
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

        // obs.correctedPose is Pedro's Pose; localizer uses Road Runner's Pose2d.
        Pose2d rr = localizer.getPose();
        lastCorrectedPose = obs.correctedPose;
        poseErrorIn = Math.hypot(
                rr.position.x - obs.correctedPose.getX(),
                rr.position.y - obs.correctedPose.getY());

        // Low-pass blend RR toward the tag estimate to reject jitter.
        double alpha = LocalizationConfig.TAG_CORRECTION_ALPHA;
        double fusedX = rr.position.x + alpha * (obs.correctedPose.getX() - rr.position.x);
        double fusedY = rr.position.y + alpha * (obs.correctedPose.getY() - rr.position.y);
        localizer.setPose(new Pose2d(fusedX, fusedY, rr.heading.toDouble()));

        // Distance-based compensation using calibrated cubic polynomials.
        // distanceIn (inches) → cm for the polynomial inputs.
        if (ShooterConfig.USE_DISTANCE_COMPENSATION
                && shooter.isAutoAimEnabled()
                && obs.distanceIn > 0) {
            double distCm = obs.distanceIn * 2.54;
            // Hood always tracks: instant servo response gives realtime visual feedback.
            hood.setPosition(ShooterConfig.hoodPitch(distCm));
            // Flywheel only overrides when the driver has already commanded it (trigger held
            // or hold mode on) — avoids spinning up unnecessarily while driving around.
            if (shooter.getTargetShooterRpm() > 0) {
                shooter.setShooterVelocityRpm(ShooterConfig.hoodTuneAngle(distCm));
            }
        }
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
        telemetry.addData("Distance", "%.1f cm  (%.1f in)",
                lastObs.distanceIn * 2.54, lastObs.distanceIn);
        telemetry.addData("Yaw (tx)", "%.1f°", lastObs.txDeg);
        telemetry.addData("Bearing (robot)", "%.1f°", lastObs.bearingRobotDeg);
        telemetry.addData("Turret Pwr", "%.3f", shooter.getLastTurretPower());

        telemetry.addLine("=== LOCALIZATION DIAGNOSTICS ===");
        telemetry.addData("RR Pose", "(%.1f, %.1f, %.1f°)",
                localizer.getXInches(), localizer.getYInches(), localizer.getHeadingDegrees());
        if (lastCorrectedPose != null) {
            telemetry.addData("Tag Corrected Pose", "(%.1f, %.1f, %.1f°)",
                    lastCorrectedPose.getX(), lastCorrectedPose.getY(),
                    Math.toDegrees(lastCorrectedPose.getHeading()));
        } else {
            telemetry.addData("Tag Corrected Pose", "-- no fix yet --");
        }
        telemetry.addData("Pose Error", "%.1f in", poseErrorIn);

        telemetry.addLine("=== DISTANCE COMPENSATION ===");
        if (lastObs.distanceIn > 0) {
            double distCm = lastObs.distanceIn * 2.54;
            telemetry.addData("Distance",    "%.1f cm  (%.1f in)", distCm, lastObs.distanceIn);
            telemetry.addData("Poly RPM",    "%.0f RPM", ShooterConfig.hoodTuneAngle(distCm));
            telemetry.addData("Poly Hood",   "%.3f", ShooterConfig.hoodPitch(distCm));
            telemetry.addData("Actual Hood", "%.3f", hood.getPosition());
            telemetry.addData("Mode", ShooterConfig.USE_DISTANCE_COMPENSATION
                    ? (shooter.isAutoAimEnabled() ? "ACTIVE" : "disabled (auto-aim off)")
                    : "OFF (toggle USE_DISTANCE_COMPENSATION)");
        } else {
            telemetry.addData("Distance", "-- no tag --");
            telemetry.addData("Mode", ShooterConfig.USE_DISTANCE_COMPENSATION ? "waiting for tag" : "OFF");
        }

        telemetry.addLine("=== HEALTH ===");
        String voltStatus = (voltage < 11.0) ? "!! BROWNOUT RISK !!" : (voltage < 12.0) ? "! LOW !" : "OK";
        String loopStatus = (loopTime > 45) ? "!! HIGH LATENCY !!" : "OK";
        telemetry.addData("Battery", "%.2fV [%s]", voltage, voltStatus);
        telemetry.addData("Loop Time", "%d ms [%s]", loopTime, loopStatus);
    }
}
