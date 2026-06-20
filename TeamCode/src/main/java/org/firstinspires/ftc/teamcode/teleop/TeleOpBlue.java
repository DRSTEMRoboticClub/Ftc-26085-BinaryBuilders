package org.firstinspires.ftc.teamcode.teleop;

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
import org.firstinspires.ftc.teamcode.tools.PriorityInputHandler;
import org.firstinspires.ftc.teamcode.tools.localization.MecanumLocalizer;

import java.util.List;

@TeleOp(name = "TeleOp Blue", group = "Main")
public class TeleOpBlue extends CommandOpMode {
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private HoodSubsystem hood;

    private PriorityInputHandler inputHandler;
    private GamepadEx g1, g2;
    private VoltageSensor batteryVoltageSensor;
    private List<LynxModule> allHubs;

    // ── Dead-reckoning localizer (fallback when LL can't see the tag) ─────────
    private MecanumLocalizer localizer;
    private int turretStartTicks;

    // ── Health / crash diagnostics ──────────────────────────────────────────
    private long lastLoopTime = 0;
    private long maxLoopMs = 0;
    private double minVoltage = 14.0;
    private int loopErrors = 0;
    private String lastError = "none";

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
        inputHandler = new PriorityInputHandler(g1, g2);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.visuallyIdentify(false);
        }

        localizer = new MecanumLocalizer(hardwareMap);
        turretStartTicks = shooter.getTurretTicks();

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

                // Localizer: integrate encoder + IMU deltas into robot pose estimate.
                // Uses heading from the previous loop (one loop old — negligible at ~50 Hz).
                localizer.update(drive.getHeading());

                // Tag correction: when the LL has a fix, blend the back-calculated robot
                // position into the localizer so dead-reckoning stays accurate after the tag
                // disappears (e.g. robot drives behind an obstacle or moves out of LL range).
                double dist  = shooter.getTrackedTagDistanceCm();
                Double tagTx = shooter.getTrackedTagTx();
                if (dist > 0 && tagTx != null) correctLocalizerFromTag(dist, tagTx);

                // Provide the dead-reckoning TX BEFORE inputHandler.update() which calls
                // runTurretControl(). When the LL sees the tag this is null (real TX takes over).
                shooter.setFallbackTx(tagTx == null ? computeDeadReckonTx() : null);

                // Distance compensation (PRE): set the flywheel RPM target from the polynomial
                // BEFORE inputHandler runs updatePID(), so the custom velocity PID uses it.
                double compDist = applyShooterCompensation();

                inputHandler.update(drive, intake, shooter, hood);

                // Distance compensation (POST): actually drive the hood servo to the polynomial
                // pitch. inputHandler's D-pad already ran, so auto wins whenever a tag is in view.
                if (compDist > 0) {
                    hood.setPosition(ShooterConfig.hoodPitch(compDist));
                }

                double voltage = batteryVoltageSensor.getVoltage();
                if (voltage < minVoltage) minVoltage = voltage;

                renderTelemetry(loopTime, voltage);
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

        String voltWarn = voltage < 11.0 ? " !!LOW!!" : voltage < 12.0 ? " !LOW!" : "";
        telemetry.addLine(String.format("BLUE | tag %d | aim %s | LL %s | %dms | %.2fV%s",
                ShooterConfig.TRACKED_TAG_ID, shooter.isAutoAimEnabled() ? "ON" : "OFF",
                shooter.isLimelightEnabled() ? "ON" : "OFF",
                loopTime, voltage, voltWarn));

        if (tx != null) {
            double normPct = Math.abs(tx) / ShooterConfig.CAMERA_HALF_FOV_DEG * 100.0;
            telemetry.addLine(String.format("LL  TAG  tx %.1f (%.0f%%)  %s  | %s",
                    tx, normPct, dist > 0 ? String.format("dist %.0fcm", dist) : "no 3D",
                    shooter.getLimelightStatus()));
        } else {
            telemetry.addLine(String.format("LL  no tag (sees %s) | %s",
                    shooter.getVisibleTagIds(), shooter.getLimelightStatus()));
        }

        if (dist > 0) {
            double d = Math.max(ShooterConfig.MIN_COMP_DISTANCE,
                    Math.min(dist, ShooterConfig.MAX_COMP_DISTANCE));
            telemetry.addLine(String.format("Poly %.0fcm -> %.0fRPM hood %.2f | act %.0fRPM hood %.2f",
                    d, ShooterConfig.hoodTuneAngle(d), ShooterConfig.hoodPitch(d),
                    shooter.getShooterVelocityRpm(), hood.getPosition()));
        } else {
            telemetry.addLine(String.format("Shooter %.0f/%.0f RPM hood %.2f",
                    shooter.getShooterVelocityRpm(), shooter.getTargetShooterRpm(), hood.getPosition()));
        }

        Pose2d pose = localizer.getPose();
        telemetry.addLine(String.format("Turret %.2f %s P=%.2f | Pose %.0f,%.0f H%.0f",
                shooter.getLastTurretPower(),
                shooter.isAutoAimEnabled() ? (shooter.getTrackedTagTx() != null ? "LL" : "DR") : "MAN",
                ShooterConfig.AUTO_AIM_P_GAIN,
                pose.position.x, pose.position.y, drive.getHeading()));

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
    private double applyShooterCompensation() {
        double dist = shooter.getTrackedTagDistanceCm();
        if (!ShooterConfig.USE_DISTANCE_COMPENSATION || dist <= 0) {
            shooter.clearAutoShootRpmOverride();   // fall back to manual RPM control
            return -1;
        }
        double d = Math.max(ShooterConfig.MIN_COMP_DISTANCE,
                            Math.min(dist, ShooterConfig.MAX_COMP_DISTANCE));

        // Only spin up to the polynomial RPM while the driver is actually requesting the shooter
        // (trigger / left bumper / hold mode) — pre-spinning would waste battery and worsen sag.
        boolean wantShoot = inputHandler.isShooterHoldMode()
                || gamepad1.left_bumper
                || gamepad1.right_trigger > ControlsConfig.TRIGGER_THRESHOLD;
        if (wantShoot) {
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
        // Current turret angle from its TeleOp start position
        double turretAngleRad = Math.toRadians(
                (shooter.getTurretTicks() - turretStartTicks)
                * LocalizationConfig.TURRET_DEG_PER_TICK
                * LocalizationConfig.TURRET_ANGLE_SIGN);
        // Camera-relative angle to tag; negate to match LL TX sign convention
        double cameraToTagRad = robotRelAngleRad - turretAngleRad;
        while (cameraToTagRad >  Math.PI) cameraToTagRad -= 2 * Math.PI;
        while (cameraToTagRad <= -Math.PI) cameraToTagRad += 2 * Math.PI;
        return -Math.toDegrees(cameraToTagRad);
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
        double turretAngleDeg  = (shooter.getTurretTicks() - turretStartTicks)
                * LocalizationConfig.TURRET_DEG_PER_TICK
                * LocalizationConfig.TURRET_ANGLE_SIGN;
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
