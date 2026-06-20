package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.geometry.Pose;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.configs.LocalizationConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.configs.ShootZoneConfig;
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
    private double lastCompDistCm = 0;
    private boolean lastCompFromTag = false;

    // ── Auto-shoot zone state ─────────────────────────────────────────────
    private boolean autoShootActive    = false;
    private boolean prevL3             = false;
    private boolean stopperCurrentOpen = false;
    private long    stopperToggledMs   = 0;
    private double  autoShootTargetRpm = 0.0; // cached for telemetry + RPM check

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

        localizer = new MecanumLocalizer(hardwareMap);
        turretTracker = new TurretTracker(shooter);

        // Make sure this OpMode tracks ITS alliance tag everywhere it matters.
        ShooterConfig.TRACKED_TAG_ID = getTagId();

        // TurretTracker is the sole auto-aim authority here — prevent runTurretControl()
        // from also driving the turret and issuing conflicting motor commands.
        shooter.setExternalTurretControl(true);

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
            // Fetch Limelight result once — all downstream methods use this cached copy.
            shooter.cacheLimelightResult();

            long currentTime = System.currentTimeMillis();
            long loopTime = (lastLoopTime == 0) ? 0 : currentTime - lastLoopTime;
            lastLoopTime = currentTime;

            // 1a) Auto-shoot: set RPM override BEFORE inputHandler so the PID in
            //     inputHandler.update() already sees the correct target this frame.
            autoShootPreUpdate();

            // 1b) Standard TeleOpBlue control (drive / intake / hood / turret / PID).
            inputHandler.update(drive, intake, shooter, hood);

            // 2) Localization subsystems.
            // Pass DriveSubsystem's cached heading — avoids a second I2C IMU read
            // and ensures the localizer + limelight MegaTag2 use the same heading.
            double headingDeg = drive.getHeading();
            localizer.update(headingDeg);
            shooter.updateLimelightOrientation(headingDeg);
            // TurretTracker only runs in auto-aim mode.
            // In manual mode (G1 X), D-pad from runTurretControl() has sole control.
            if (shooter.isAutoAimEnabled()) {
                try {
                    turretTracker.update(shooter, getTagId());
                } catch (Exception e) {
                    // Ignore bad frames — turret holds last power
                }
            }
            try {
                applyAprilTagCorrection();   // updates lastObs for this frame
            } catch (Exception e) {
                // Ignore malformed Limelight pose data rather than crashing
            }

            // 1c) Auto-shoot stopper — runs AFTER inputHandler so we override its
            //     stopper setting, and AFTER applyAprilTagCorrection for fresh distance.
            autoShootStopper();

            // 3) Field visualisation — non-blocking, PanelsFieldDrawer handles rate-limiting
            //    and network I/O on its own daemon thread.
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

        // ── Pose correction: only when we have a trusted tag measurement ────
        if (obs.correctedPose != null) {
            Pose2d rr = localizer.getPose();
            lastCorrectedPose = obs.correctedPose;
            poseErrorIn = Math.hypot(
                    rr.position.x - obs.correctedPose.getX(),
                    rr.position.y - obs.correctedPose.getY());

            double alpha = LocalizationConfig.TAG_CORRECTION_ALPHA;
            double fusedX = rr.position.x + alpha * (obs.correctedPose.getX() - rr.position.x);
            double fusedY = rr.position.y + alpha * (obs.correctedPose.getY() - rr.position.y);
            localizer.setPose(new Pose2d(fusedX, fusedY, rr.heading.toDouble()));
        }

        // ── Distance compensation: tag distance first, localizer pose as fallback ──
        // The localizer fallback lets the polynomial keep running even when the
        // Limelight temporarily loses the tag (e.g. turret mid-swing, occlusion).
        if (ShooterConfig.USE_DISTANCE_COMPENSATION && shooter.isAutoAimEnabled()) {
            double distCm;
            if (obs.distanceIn > 0) {
                distCm = obs.distanceIn;        // live AprilTag measurement (cm)
                lastCompFromTag = true;
            } else {
                // Dead-reckoning fallback: distance from current pose to known tag position.
                // Both localizer and tag field positions share the same cm coordinate frame.
                distCm = Math.hypot(localizer.getXInches() - getTagFieldX(),
                                    localizer.getYInches() - getTagFieldY());
                lastCompFromTag = false;
            }
            // Clamp to calibration range — outside [MIN,MAX] the polynomial extrapolates badly.
            distCm = Math.max(ShooterConfig.MIN_COMP_DISTANCE,
                              Math.min(distCm, ShooterConfig.MAX_COMP_DISTANCE));
            lastCompDistCm = distCm;

            hood.setPosition(ShooterConfig.hoodPitch(distCm));
            if (shooter.getTargetShooterRpm() > 0) {
                shooter.setShooterVelocityRpm(ShooterConfig.hoodTuneAngle(distCm));
            }
        }
    }

    // ── Auto-shoot helpers ────────────────────────────────────────────────────

    /**
     * Call BEFORE inputHandler.update() so the RPM override is visible to the
     * updatePID() call that runs inside inputHandler.
     */
    private void autoShootPreUpdate() {
        boolean l3 = gamepad1.left_stick_button;
        if (l3 && !prevL3) {
            autoShootActive = !autoShootActive;
            if (!autoShootActive) {
                shooter.clearAutoShootRpmOverride();
                autoShootTargetRpm = 0.0;
            }
        }
        prevL3 = l3;

        if (!autoShootActive) return;

        // Tag distance first; localizer dead-reckoning as fallback when tag not visible.
        if (ShooterConfig.USE_DISTANCE_COMPENSATION) {
            double distCm = (lastObs.distanceIn > 0)
                    ? lastObs.distanceIn
                    : Math.hypot(localizer.getXInches() - getTagFieldX(),
                                 localizer.getYInches() - getTagFieldY());
            distCm = Math.max(ShooterConfig.MIN_COMP_DISTANCE,
                              Math.min(distCm, ShooterConfig.MAX_COMP_DISTANCE));
            autoShootTargetRpm = ShooterConfig.hoodTuneAngle(distCm);
        } else {
            autoShootTargetRpm = ShooterConfig.MANUAL_TARGET_RPM;
        }
        shooter.setAutoShootRpmOverride(autoShootTargetRpm);
    }

    /**
     * Call AFTER inputHandler.update() and applyAprilTagCorrection().
     * Cycles the stopper open/close when in a shooting zone and RPM is on target.
     */
    private void autoShootStopper() {
        if (!autoShootActive) {
            if (stopperCurrentOpen) {
                shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
                stopperCurrentOpen = false;
            }
            return;
        }

        if (!isInShootingZone() || !isShooterReadyForAutoShoot()) {
            if (stopperCurrentOpen) {
                shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
                stopperCurrentOpen = false;
                stopperToggledMs = System.currentTimeMillis();
            }
            return;
        }

        long now = System.currentTimeMillis();
        long elapsed = now - stopperToggledMs;

        if (stopperCurrentOpen) {
            if (elapsed >= ShootZoneConfig.STOPPER_OPEN_MS) {
                shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
                stopperCurrentOpen = false;
                stopperToggledMs = now;
            }
        } else {
            if (elapsed >= ShootZoneConfig.STOPPER_CLOSE_MS) {
                shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
                stopperCurrentOpen = true;
                stopperToggledMs = now;
            }
        }
    }

    private boolean isInShootingZone() {
        double x = localizer.getXInches();
        double y = localizer.getYInches();
        boolean z1 = x >= ShootZoneConfig.ZONE1_X_MIN && x <= ShootZoneConfig.ZONE1_X_MAX
                && y >= ShootZoneConfig.ZONE1_Y_MIN && y <= ShootZoneConfig.ZONE1_Y_MAX;
        boolean z2 = x >= ShootZoneConfig.ZONE2_X_MIN && x <= ShootZoneConfig.ZONE2_X_MAX
                && y >= ShootZoneConfig.ZONE2_Y_MIN && y <= ShootZoneConfig.ZONE2_Y_MAX;
        return z1 || z2;
    }

    private boolean isShooterReadyForAutoShoot() {
        if (autoShootTargetRpm <= 0) return false;
        return Math.abs(shooter.getShooterVelocityRpm() - autoShootTargetRpm)
                <= ShootZoneConfig.SHOOT_READY_RPM_TOLERANCE;
    }

    private void renderTelemetry(double voltage, long loopTime) {
        // ── Header: always-visible summary ───────────────────────────────────
        String voltWarn = voltage < 11.0 ? " !!LOW!!" : voltage < 12.0 ? " !LOW!" : "";
        telemetry.addLine(String.format("%-5s | tag %d | aim %-3s | %dms | %.2fV%s",
                getAllianceName(), getTagId(),
                shooter.isAutoAimEnabled() ? "ON" : "OFF",
                loopTime, voltage, voltWarn));

        // ── Pose ─────────────────────────────────────────────────────────────
        telemetry.addLine(String.format("Pose  X:%.1f  Y:%.1f  H:%.1f°",
                localizer.getXInches(), localizer.getYInches(), localizer.getHeadingDegrees()));
        if (lastCorrectedPose != null) {
            telemetry.addLine(String.format("Fix   (%.1f, %.1f)  err %.1f cm",
                    lastCorrectedPose.getX(), lastCorrectedPose.getY(), poseErrorIn));
        } else {
            telemetry.addLine("Fix   -- no AprilTag fix yet --");
        }

        // ── Turret ───────────────────────────────────────────────────────────
        telemetry.addLine(String.format("Turret  %.1f°  %-8s  pwr %.2f",
                turretTracker.getLastTurretAngleDegrees(),
                turretTracker.isUnwinding() ? "FLIPPING" : "tracking",
                shooter.getLastTurretPower()));

        // ── Limelight: distance + aim offset ─────────────────────────────────
        if (lastObs.visible) {
            // Show normalised offset so you can see how the proportional tracking scales:
            // 0% = tag perfectly centred in camera; 100% = tag at edge of FOV.
            double normPct = Math.abs(lastObs.txDeg) / LocalizationConfig.CAMERA_HALF_FOV_DEG * 100.0;
            telemetry.addLine(String.format("LL  TAG SEEN  tx %.1f° (%.0f%%)  dist %.0f cm",
                    lastObs.txDeg, normPct, lastObs.distanceIn));
        } else {
            telemetry.addLine("LL  no tag");
        }

        // ── Shooter polynomial (the main display the user requested) ─────────
        if (ShooterConfig.USE_DISTANCE_COMPENSATION && shooter.isAutoAimEnabled()) {
            String src = lastCompFromTag ? "tag" : "DR";  // DR = dead-reckoning
            double polyRpm  = ShooterConfig.hoodTuneAngle(lastCompDistCm);
            double polyHood = ShooterConfig.hoodPitch(lastCompDistCm);
            telemetry.addLine(String.format("Dist  %.0f cm (%s)  →  %.0f RPM  hood %.2f",
                    lastCompDistCm, src, polyRpm, polyHood));
            telemetry.addLine(String.format("Actual  %.0f / %.0f RPM  hood %.2f",
                    shooter.getShooterVelocityRpm(), polyRpm, hood.getPosition()));
        } else {
            telemetry.addLine(String.format("Shooter  %.0f RPM  hood %.2f",
                    shooter.getShooterVelocityRpm(), hood.getPosition()));
        }

        // ── Auto-shoot ───────────────────────────────────────────────────────
        if (autoShootActive) {
            telemetry.addLine(String.format("AutoShoot ON  %s  stopper %s",
                    isInShootingZone() ? "IN ZONE" : "out of zone",
                    stopperCurrentOpen ? "OPEN" : "closed"));
        } else {
            telemetry.addLine("AutoShoot OFF  (G1 L3)");
        }
    }
}
