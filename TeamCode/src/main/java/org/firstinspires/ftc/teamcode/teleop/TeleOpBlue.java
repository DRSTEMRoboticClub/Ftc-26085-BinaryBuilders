package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.configs.ControlsConfig;
import org.firstinspires.ftc.teamcode.configs.LocalizationConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.teleop.subsystems.*;
import org.firstinspires.ftc.teamcode.tools.InputHandler;
import org.firstinspires.ftc.teamcode.tools.localization.MecanumLocalizer;

import java.util.List;

@TeleOp(name = "TeleOp Blue", group = "Main")
public class TeleOpBlue extends CommandOpMode {
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private HoodSubsystem hood;

    private InputHandler inputHandler;
    private GamepadEx g1, g2;
    private VoltageSensor batteryVoltageSensor;
    private List<LynxModule> allHubs;

    // ── Dead-reckoning localizer (fallback when LL can't see the tag) ─────────
    private MecanumLocalizer localizer;

    // ── Health / crash diagnostics ──────────────────────────────────────────
    private long lastLoopTime = 0;
    private long maxLoopMs = 0;
    private long lastTelemetryRenderMs = 0;
    private double minVoltage = 14.0;
    private int loopErrors = 0;
    private String lastError = "none";

    // GC / heap diagnostics — heap drop while tag visible = GC pressure from LL SDK.
    // longLoopCount = loops that took >30 ms (major GC pauses stall the thread for 50–500 ms).
    private int longLoopCount = 0;
    private long heapFreeMb = 0;
    private long heapTotalMb = 0;
    // Tag-visibility streak: tracks how many seconds the LL has been seeing tag 20 continuously.
    // Disconnect typically happens at 5–6 s; this number tells us exactly when it happened.
    private long tagVisibleSinceMs = 0;
    // Logcat logging fires every 500 ms so the data is readable via ADB after a crash.
    private long lastLogMs = 0;

    // ── Long-range localizer fallback ────────────────────────────────────────
    // True once correctLocalizerFromTag() has been called at least once, meaning the
    // localizer pose has been seeded by a close-range LL fix and can be trusted for
    // computing distance + TX when the tag is beyond LL_FALLBACK_DISTANCE_CM.
    private boolean localizerCalibrated = false;
    // Set each loop — passed into renderTelemetry() to label source on the DS.
    private double effectiveDist = -1.0;
    private boolean usingLocFallback = false;

    // Captures uncaught crashes from EVERY thread (the Limelight SDK's polling thread, any
    // leftover daemon threads, the main loop). catch(Throwable) below only protects the main
    // loop; a background thread dying — or a main-thread Error like OutOfMemoryError, which is
    // NOT an Exception — would otherwise vanish with no trace. This surfaces both.
    private static volatile String lastUncaught = "none";
    private Thread.UncaughtExceptionHandler previousHandler;

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        hood = new HoodSubsystem(hardwareMap);

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        inputHandler = new InputHandler(g1, g2);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.visuallyIdentify(false);
        }

        localizer = new MecanumLocalizer(hardwareMap);

        // TRACKED_TAG_ID is static and overwritten per alliance by LocalSys — force it back.
        ShooterConfig.TRACKED_TAG_ID = 20;

        // Record uncaught crashes from any thread, then delegate to the SDK's own handler so
        // its crash reporting/logging still runs. Restored on stop to avoid leaking global state.
        previousHandler = Thread.getDefaultUncaughtExceptionHandler();
        Thread.setDefaultUncaughtExceptionHandler((thread, throwable) -> {
            lastUncaught = thread.getName() + " / " + throwable.getClass().getSimpleName()
                    + (throwable.getMessage() != null ? ": " + throwable.getMessage() : "");
            if (previousHandler != null) previousHandler.uncaughtException(thread, throwable);
        });
    }

    @Override
    public void runOpMode() {
        initialize();
        // Throttle telemetry to ~10 Hz. Pushing it every loop (with FTC Dashboard mirroring over
        // WiFi) can saturate the link and drop the Driver Station connection.
        telemetry.setMsTransmissionInterval(100);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            try {
                for (LynxModule hub : allHubs) hub.clearBulkCache();
                shooter.cacheLimelightResult();

                long now = System.currentTimeMillis();
                long loopTime = (lastLoopTime == 0) ? 0 : now - lastLoopTime;
                lastLoopTime = now;
                if (loopTime > maxLoopMs) maxLoopMs = loopTime;
                if (loopTime > 30) longLoopCount++;

                // Sample heap state (cheap — reads JVM counters only).
                Runtime rt = Runtime.getRuntime();
                heapFreeMb  = rt.freeMemory()  >> 20;
                heapTotalMb = rt.totalMemory() >> 20;

                // Localizer: integrate encoder + IMU deltas into robot pose estimate.
                // Uses heading from the previous loop (one loop old — negligible at ~50 Hz).
                localizer.update(drive.getHeading());

                // ── LL data + localizer correction ───────────────────────────────────
                double dist  = shooter.getTrackedTagDistanceCm();
                Double tagTx = shooter.getTrackedTagTx();

                // Only seed the localizer with LL fixes when the tag is close enough for the
                // 360x240 resolution to give accurate TX/TY readings. Beyond LL_FALLBACK_DISTANCE_CM
                // the pixel measurements degrade and would introduce noisy corrections.
                boolean tagCloseEnough = (dist > 0 && dist <= ShooterConfig.LL_FALLBACK_DISTANCE_CM);
                if (tagCloseEnough && tagTx != null) {
                    correctLocalizerFromTag(dist, tagTx);
                    localizerCalibrated = true;
                }

                // ── Effective TX + distance selection ────────────────────────────────
                // Close range  (<= LL_FALLBACK_DISTANCE_CM): raw LL TX and TY distance.
                // Long range   (>  LL_FALLBACK_DISTANCE_CM): dead-reckoning TX, localizer
                //   distance — both derived from the robot's field-position estimate, so
                //   they are resolution-independent and stable at any range.
                Double effectiveTx;
                if (tagCloseEnough) {
                    effectiveTx  = tagTx;
                    effectiveDist = dist;
                    usingLocFallback = false;
                } else if (localizerCalibrated) {
                    effectiveTx  = null;          // null forces dead-reckoning in runTurretControl
                    effectiveDist = computeLocalizerDistanceCm();
                    usingLocFallback = true;
                } else {
                    // Localizer not yet seeded — use raw LL values even if imprecise.
                    effectiveTx  = tagTx;
                    effectiveDist = dist;
                    usingLocFallback = false;
                }

                // Track continuous tag-visible streak for disconnect correlation.
                if (tagTx != null) {
                    if (tagVisibleSinceMs == 0) tagVisibleSinceMs = now;
                } else {
                    tagVisibleSinceMs = 0;
                }
                long tagStreakMs = (tagVisibleSinceMs == 0) ? 0 : now - tagVisibleSinceMs;

                // Log to logcat every 500 ms — survives a DS disconnect, readable via ADB after.
                // Command:  adb logcat -d | grep TELE_BLUE
                if (now - lastLogMs >= 500) {
                    lastLogMs = now;
                    Log.d("TELE_BLUE", String.format(
                        "loop=%dms max=%dms gc>30ms=%d heap=%d/%dMB tag=%.1fs dist=%.0fcm(%s) ll=%s err=%d",
                        loopTime, maxLoopMs, longLoopCount,
                        heapFreeMb, heapTotalMb,
                        tagStreakMs / 1000.0, effectiveDist,
                        usingLocFallback ? "LOC" : "LL",
                        shooter.getLimelightStatus(), loopErrors));
                }


                // Distance compensation (PRE): set the flywheel RPM target from the polynomial
                // BEFORE inputHandler runs updatePID(), so the custom velocity PID uses it.
                // In manual mode, G2 D-pad controls RPM directly — skip the polynomial override.
                double compDist;
                if (inputHandler.isManualMode()) {
                    shooter.clearAutoShootRpmOverride();
                    compDist = -1;
                } else {
                    compDist = applyShooterCompensation(effectiveDist);
                }

                // Feed live heading to the turret so it counter-rotates during chassis turns
                // (heading feed-forward). Must run before inputHandler -> runTurretControl().
                shooter.setRobotHeading(drive.getHeading());

                inputHandler.update(drive, intake, shooter, hood);

                // Hood control: auto mode (X=off) → polynomial from distance;
                // manual mode (X=on) → G2 D-pad only (inputHandler already ran above).
                if (!inputHandler.isManualMode() && compDist > 0) {
                    hood.setPosition(ShooterConfig.hoodPitch(compDist));
                }

                double voltage = batteryVoltageSensor.getVoltage();
                if (voltage < minVoltage) minVoltage = voltage;

                // renderTelemetry() builds strings every call; gate it to the actual DS
                // transmission interval so String.format() allocations don't run every loop.
                if (now - lastTelemetryRenderMs >= 100) {
                    renderTelemetry(loopTime, voltage);
                    lastTelemetryRenderMs = now;
                }
            } catch (Throwable t) {
                // Catch Throwable (not just Exception) so an Error can't silently end runOpMode().
                loopErrors++;
                lastError = t.getClass().getSimpleName()
                        + (t.getMessage() != null ? ": " + t.getMessage() : "");
                safeActuators();
            } finally {
                try { telemetry.update(); } catch (Throwable ignored) { }
            }
        }

        safeActuators();
        shooter.stopLimelight();
        Thread.setDefaultUncaughtExceptionHandler(previousHandler);
    }

    private void renderTelemetry(long loopTime, double voltage) {
        Double tx   = shooter.getTrackedTagTx();
        double dist = shooter.getTrackedTagDistanceCm();
        long tagStreakMs = (tagVisibleSinceMs == 0) ? 0 : System.currentTimeMillis() - tagVisibleSinceMs;

        String voltWarn = voltage < 11.0 ? " !!LOW!!" : voltage < 12.0 ? " !LOW!" : "";
        telemetry.addLine(String.format("BLUE | tag %d | aim %s | hood %s | LL %s | %dms | %.2fV (min %.2fV)%s",
                ShooterConfig.TRACKED_TAG_ID, shooter.isAutoAimEnabled() ? "ON" : "OFF",
                inputHandler.isManualMode() ? "MANUAL(G2)" : "AUTO(poly)",
                shooter.isLimelightEnabled() ? "ON" : "OFF",
                loopTime, voltage, minVoltage, voltWarn));

        if (tx != null) {
            double normPct = Math.abs(tx) / ShooterConfig.CAMERA_HALF_FOV_DEG * 100.0;
            String distStr = effectiveDist > 0
                    ? String.format("dist %.0fcm %s", effectiveDist, usingLocFallback ? "[LOC]" : "[LL]")
                    : "no dist";
            telemetry.addLine(String.format("LL  TAG  tx %.1f (%.0f%%)  %s  | vis %.1fs | %s",
                    tx, normPct, distStr, tagStreakMs / 1000.0,
                    shooter.getLimelightStatus()));
        } else {
            String src = usingLocFallback ? "[LOC dist]" : "";
            telemetry.addLine(String.format("LL  no lock %s| %s",
                    src, shooter.getLimelightStatus()));
        }
        // Always-on LL diagnostics — tells you WHY there is no lock:
        //   fids=0            → LL sees no AprilTags (wrong pipeline? not pointed at a tag?)
        //   fids>0 ids=[24]   → it sees tag 24 but track=20 → TRACKED_TAG_ID mismatch
        //   valid=false       → result invalid (LL still booting / bad frame)
        //   "not started"     → LL off (G2-X toggle) or USB not enumerated
        telemetry.addLine("LLdbg " + shooter.getLimelightDebugInfo());
        telemetry.addLine("LLpipe " + shooter.getPipelineUploadStatus());

        telemetry.addLine(String.format("RPM  NEAR %.0f (↑↓)  FAR %.0f (←→)",
                ShooterConfig.NEAR_RPM, ShooterConfig.FAR_RPM));
        // Manual tuning readout (G2 D-pad L/R = power, U/D = hood).
        telemetry.addLine(String.format("MANUAL  power(RPM) %.0f -> act %.0f  |  hood %.3f",
                ShooterConfig.MANUAL_TARGET_RPM, shooter.getShooterVelocityRpm(),
                hood.getPosition()));

        if (effectiveDist > 0) {
            double d = Math.max(ShooterConfig.MIN_COMP_DISTANCE,
                    Math.min(effectiveDist, ShooterConfig.MAX_COMP_DISTANCE));
            double targetRpm = ShooterConfig.hoodTuneAngle(d);
            telemetry.addLine(String.format("Poly %.0fcm -> %.0f RPM | L=%.0f R=%.0f raw=%.0f t/s",
                    d, targetRpm,
                    shooter.getLeftShooterRpm(), shooter.getRightShooterRpm(),
                    shooter.getRawLauncherTicksPerSec()));
            telemetry.addLine(String.format("Hood poly=%.2f actual=%.2f | CPR=%d",
                    ShooterConfig.hoodPitch(d), hood.getPosition(),
                    (int) ShooterConfig.SHOOTER_ENCODER_EVENTS_PER_REV));
        } else {
            telemetry.addLine(String.format("Shooter L=%.0f R=%.0f target=%.0f RPM | raw=%.0f t/s",
                    shooter.getLeftShooterRpm(), shooter.getRightShooterRpm(),
                    shooter.getEffectiveTargetRpm(), shooter.getRawLauncherTicksPerSec()));
        }

        Pose2d pose = localizer.getPose();
        String turretSrc = !shooter.isAutoAimEnabled()       ? "MAN"
                : shooter.isTurretAtLimit()                  ? "LIMIT"
                : shooter.isTurretSearching()                ? "SEEK"
                : shooter.getTrackedTagTx() != null          ? "LL"
                : "HOLD";
        double turretAngleDeg = shooter.getTurretAngleDeg();
        String limitWarn = Math.abs(turretAngleDeg) >= LocalizationConfig.TURRET_FLIP_ANGLE * 0.85
                ? " !LIMIT!" : "";
        telemetry.addLine(String.format("Turret %.2f %s | %.1f° (ticks %d) flip@%.0f°%s",
                shooter.getLastTurretPower(), turretSrc,
                turretAngleDeg, shooter.getTurretTicks(),
                LocalizationConfig.TURRET_FLIP_ANGLE, limitWarn));
        telemetry.addLine(String.format("Pose %.0f,%.0f H%.0f | P=%.3f I=%.3f D=%.4f",
                pose.position.x, pose.position.y, drive.getHeading(),
                ShooterConfig.TURRET_P, ShooterConfig.TURRET_I, ShooterConfig.TURRET_D));

        // Heap + GC diagnostics — heap drops while tag visible → GC pressure from LL SDK.
        // gc>30ms rises steadily → GC pauses are the disconnect cause.
        // If heap is stable but gc>30ms still rises → something else is stalling the loop.
        telemetry.addLine(String.format("Heap %dMB free / %dMB alloc | gc>30ms: %d loops",
                heapFreeMb, heapTotalMb, longLoopCount));

        telemetry.addLine(String.format("Health maxLoop %dms minV %.1f err %d (%s)",
                maxLoopMs, minVoltage, loopErrors, lastError));
        telemetry.addLine("Uncaught: " + lastUncaught);
    }

    /**
     * Feeds the distance polynomial into the flywheel via the custom velocity PID (never raw
     * setPower). When a tag is visible and the driver is requesting the shooter, the flywheel
     * target RPM is overridden to hoodTuneAngle(distance); updatePID() (run inside inputHandler)
     * honors the override. When no tag / not shooting, the override is cleared so the normal
     * manual RPM path takes over.
     *
     * @return the clamped distance (cm) if compensation is active this loop, else -1.
     */
    private double applyShooterCompensation(double dist) {
        if (!ShooterConfig.USE_DISTANCE_COMPENSATION || dist <= 0) {
            shooter.clearAutoShootRpmOverride();   // fall back to manual RPM control
            return -1;
        }
        double d = Math.max(ShooterConfig.MIN_COMP_DISTANCE,
                            Math.min(dist, ShooterConfig.MAX_COMP_DISTANCE));

        // Only spin up to the polynomial RPM while autoaim is active (G2 left trigger held).
        boolean wantShoot = inputHandler.isShooterHoldMode();
        if (wantShoot) {
            // Raw polynomial RPM — no added boost.
            shooter.setAutoShootRpmOverride(ShooterConfig.hoodTuneAngle(d));
        } else {
            shooter.clearAutoShootRpmOverride();
        }
        return d;
    }

    /**
     * Computes a synthetic TX (degrees) from dead reckoning when the LL can't see the tag.
     *
     * Math:
     *   1. Vector from robot position to known tag field position.
     *   2. Convert to robot-relative angle (subtract robot heading).
     *   3. Subtract current turret angle so the result is camera-relative.
     *   4. Negate: RR uses +Y=left (CCW positive), LL TX uses +right (CW positive).
     *
     * The result feeds into the same PD controller as a real TX. When the tag
     * re-appears in the LL frame, setFallbackTx(null) restores the real TX path.
     */
    private Double computeDeadReckonTx() {
        Pose2d pose = localizer.getPose();
        double dx = LocalizationConfig.BLUE_TAG_FIELD_X - pose.position.x;
        double dy = LocalizationConfig.BLUE_TAG_FIELD_Y - pose.position.y;
        double fieldAngleRad    = Math.atan2(dy, dx);
        double robotRelAngleRad = fieldAngleRad - pose.heading.toDouble();
        // Normalise to (-π, π]
        while (robotRelAngleRad >  Math.PI) robotRelAngleRad -= 2 * Math.PI;
        while (robotRelAngleRad <= -Math.PI) robotRelAngleRad += 2 * Math.PI;
        double turretAngleRad = Math.toRadians(shooter.getTurretAngleDeg());
        // Camera-relative angle to tag; negate to match LL TX sign convention
        double cameraToTagRad = robotRelAngleRad - turretAngleRad;
        while (cameraToTagRad >  Math.PI) cameraToTagRad -= 2 * Math.PI;
        while (cameraToTagRad <= -Math.PI) cameraToTagRad += 2 * Math.PI;
        return -Math.toDegrees(cameraToTagRad);
    }

    /**
     * Horizontal distance (cm) from the robot's localizer-estimated position to the
     * alliance tag. Used instead of TY-based LL distance beyond LL_FALLBACK_DISTANCE_CM
     * where the 360x240 resolution is no longer accurate enough for reliable shooter
     * compensation. Requires the localizer to have been seeded by at least one close-range
     * LL fix (localizerCalibrated = true) before this value is trusted.
     */
    private double computeLocalizerDistanceCm() {
        Pose2d pose = localizer.getPose();
        double dx = LocalizationConfig.BLUE_TAG_FIELD_X - pose.position.x;
        double dy = LocalizationConfig.BLUE_TAG_FIELD_Y - pose.position.y;
        return Math.hypot(dx, dy);
    }

    /**
     * Blends a tag-observation-derived robot position into the localizer pose so
     * dead reckoning stays calibrated while the LL has a fix.
     *
     * Robot position is back-calculated from: known tag field pos, measured distance,
     * current TX, robot heading, and current turret angle.
     * The correction is blended with TAG_CORRECTION_ALPHA (0.25) to avoid jumps.
     */
    private void correctLocalizerFromTag(double dist, double txDeg) {
        Pose2d cur = localizer.getPose();
        double robotHeadingRad = cur.heading.toDouble();
        double turretAngleDeg = shooter.getTurretAngleDeg();
        // Direction from camera to tag in field frame.
        // TX positive = tag right in LL = clockwise = negative in RR (+Y=left).
        double dirRad = robotHeadingRad + Math.toRadians(turretAngleDeg) - Math.toRadians(txDeg);
        // Estimated robot field position (camera offset ignored — ~3% error at 300 cm)
        double estX = LocalizationConfig.BLUE_TAG_FIELD_X - dist * Math.cos(dirRad);
        double estY = LocalizationConfig.BLUE_TAG_FIELD_Y - dist * Math.sin(dirRad);
        double a = LocalizationConfig.TAG_CORRECTION_ALPHA;
        localizer.setPose(new Pose2d(
                new Vector2d(cur.position.x + a * (estX - cur.position.x),
                             cur.position.y + a * (estY - cur.position.y)),
                robotHeadingRad));
    }

    private void safeActuators() {
        try {
            shooter.setShooterVelocityRpm(0);
            shooter.updatePID();
            shooter.setTurretPower(0);
            intake.setPower(0);
        } catch (Throwable ignored) { }
    }
}
