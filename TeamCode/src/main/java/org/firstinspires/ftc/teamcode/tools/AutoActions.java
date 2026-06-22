package org.firstinspires.ftc.teamcode.tools;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autoop.PedroAutoRunner;
import org.firstinspires.ftc.teamcode.configs.IntakeConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.teleop.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.ShooterSubsystem;

/**
 * Blocking / timed action helpers for use inside autonomous OpModes.
 *
 * Every method that loops internally:
 *   - Checks opMode.opModeIsActive() + opMode.isStopRequested() every iteration
 *     so the auto can be safely cancelled at any time.
 *   - Calls shooter.updatePID() when a shooter is involved, so the flywheel PID
 *     stays alive throughout the action (forgetting this is a common cause of
 *     the flywheel stopping unexpectedly).
 *   - Calls runner.update() / follower.update() when path-following is involved.
 *
 * Usage example — spin up, follow a path, fire:
 *
 *   AutoActions.spinUp(this, shooter, 3500, 3000);
 *   AutoActions.followPath(this, runner, shooter, 5000);
 *   AutoActions.fire(this, shooter, 600);
 *   AutoActions.spinDown(shooter);
 *
 * Usage example — vision-guided ball chase:
 *
 *   ll.switchToColor();
 *   follower.startTeleopDrive();
 *   AutoActions.chaseBlob(this, intake, follower, ll, 5000, 3.0);
 *   follower.breakFollowing();
 *   ll.switchToAprilTag();
 */
public final class AutoActions {

    private AutoActions() {}

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // SHOOTER
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /**
     * Spin the flywheel to {@code targetRpm} and block until it reaches that speed
     * (within {@link ShooterConfig#SHOOT_RPM_TOLERANCE} RPM), or until
     * {@code timeoutMs} elapses.
     *
     * @return true if the shooter reached target speed, false if it timed out
     */
    public static boolean spinUp(LinearOpMode opMode,
                                 ShooterSubsystem shooter,
                                 double targetRpm,
                                 long timeoutMs) {
        shooter.setShooterVelocityRpm(targetRpm);
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            shooter.updatePID();
            double actual = shooter.getShooterVelocityRpm();
            if (Math.abs(actual - targetRpm) <= ShooterConfig.RPM_TUNE_STEP_FINE * 4) return true;
            if (System.currentTimeMillis() >= deadline) return false;
        }
        return false;
    }

    /**
     * Spin up using {@link ShooterConfig#MANUAL_TARGET_RPM} with a 3-second timeout.
     * Convenience overload for the common case.
     */
    public static boolean spinUp(LinearOpMode opMode, ShooterSubsystem shooter) {
        return spinUp(opMode, shooter, ShooterConfig.MANUAL_TARGET_RPM, 3000);
    }

    /**
     * Set the flywheel target to 0 and stop the PID output.
     */
    public static void spinDown(ShooterSubsystem shooter) {
        shooter.setShooterVelocityRpm(0);
        shooter.updatePID();
    }

    /**
     * Open the stopper for {@code fireMs} milliseconds, then close it.
     * Keeps the flywheel PID running throughout so speed is maintained during fire.
     *
     * @param fireMs how long the stopper stays open (milliseconds)
     */
    public static void fire(LinearOpMode opMode,
                            ShooterSubsystem shooter,
                            long fireMs) {
        shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
        long deadline = System.currentTimeMillis() + fireMs;
        while (opMode.opModeIsActive() && !opMode.isStopRequested()
                && System.currentTimeMillis() < deadline) {
            shooter.updatePID();
        }
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
    }

    /**
     * Complete shoot sequence: spin up → fire → close stopper.
     * The flywheel is NOT spun down afterward so you can chain multiple shots.
     * Call {@link #spinDown} when done.
     *
     * @param targetRpm        desired flywheel speed
     * @param spinupTimeoutMs  give up waiting for speed after this many ms
     * @param fireMs           stopper-open duration in ms
     * @return true if the shooter reached speed before firing, false if it timed out
     */
    public static boolean shoot(LinearOpMode opMode,
                                ShooterSubsystem shooter,
                                double targetRpm,
                                long spinupTimeoutMs,
                                long fireMs) {
        boolean atSpeed = spinUp(opMode, shooter, targetRpm, spinupTimeoutMs);
        fire(opMode, shooter, fireMs);
        return atSpeed;
    }

    /**
     * Shoot using default config values ({@link ShooterConfig#MANUAL_TARGET_RPM},
     * 3 s spin-up timeout, 600 ms fire window).
     */
    public static boolean shoot(LinearOpMode opMode, ShooterSubsystem shooter) {
        return shoot(opMode, shooter, ShooterConfig.MANUAL_TARGET_RPM, 3000, 600);
    }

    /**
     * Shoot using the distance-compensated RPM from the polynomial, then fire.
     * Hood is also set to the calibrated position for that distance.
     *
     * @param distanceCm camera-to-target planar distance in centimetres
     */
    public static boolean shootAtDistance(LinearOpMode opMode,
                                          ShooterSubsystem shooter,
                                          HoodSubsystem hood,
                                          double distanceCm,
                                          long spinupTimeoutMs,
                                          long fireMs) {
        hood.setPosition(ShooterConfig.hoodPitch(distanceCm));
        double targetRpm = ShooterConfig.hoodTuneAngle(distanceCm);
        return shoot(opMode, shooter, targetRpm, spinupTimeoutMs, fireMs);
    }

    /**
     * Pulse the stopper N times with a short open/close cycle — useful for
     * bouncing a stuck ball through.
     *
     * @param pulses    number of open-close cycles
     * @param openMs    stopper-open duration per pulse (ms)
     * @param closeMs   stopper-closed gap between pulses (ms)
     */
    public static void pulseStopper(LinearOpMode opMode,
                                    ShooterSubsystem shooter,
                                    int pulses,
                                    long openMs,
                                    long closeMs) {
        for (int i = 0; i < pulses && opMode.opModeIsActive(); i++) {
            shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
            sleepWithPid(opMode, shooter, openMs);
            shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
            if (i < pulses - 1) sleepWithPid(opMode, shooter, closeMs);
        }
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // INTAKE
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /**
     * Run the intake forward for {@code durationMs}, then stop.
     * Uses the full forward power from {@link IntakeConfig#INTAKE_FWD_POWER}.
     */
    public static void intake(LinearOpMode opMode,
                              IntakeSubsystem intake,
                              long durationMs) {
        intake.setPower(IntakeConfig.INTAKE_FWD_POWER);
        opMode.sleep(durationMs);
        intake.setPower(0);
    }

    /**
     * Run the intake forward for {@code durationMs} while also running the
     * flywheel PID. Use this when the shooter is spinning and you need to
     * keep the PID alive during intake.
     */
    public static void intakeWithPid(LinearOpMode opMode,
                                     IntakeSubsystem intake,
                                     ShooterSubsystem shooter,
                                     long durationMs) {
        intake.setPower(IntakeConfig.INTAKE_FWD_POWER);
        sleepWithPid(opMode, shooter, durationMs);
        intake.setPower(0);
    }

    /**
     * Run the intake in reverse for {@code durationMs} to clear a jam,
     * then stop.
     */
    public static void reverseIntake(LinearOpMode opMode,
                                     IntakeSubsystem intake,
                                     long durationMs) {
        intake.setPower(IntakeConfig.INTAKE_REV_POWER);
        opMode.sleep(durationMs);
        intake.setPower(0);
    }

    /**
     * Vision-guided ball chase using the Limelight colour-blob pipeline.
     * Steers the robot using Pedro's teleopDrive toward the largest detected blob.
     * The caller is responsible for:
     *   - Switching the Limelight to the colour pipeline before calling.
     *   - Calling {@code follower.startTeleopDrive()} before calling.
     *   - Calling {@code follower.breakFollowing()} after returning.
     *
     * @param follower         Pedro follower in teleop-drive mode
     * @param ll               LimelightManager on the colour-blob pipeline
     * @param timeoutMs        give up after this many ms
     * @param areaThreshold    blob area % at which ball is considered captured
     * @param txDeadbandDeg    TX error within which we drive forward (else just turn)
     * @param turnGain         proportional turn gain (power per degree of TX error)
     * @param drivePower       forward speed when ball is centred
     * @return true if ball captured (area exceeded threshold), false if timed out
     */
    public static boolean chaseBlob(LinearOpMode opMode,
                                    IntakeSubsystem intake,
                                    Follower follower,
                                    LimelightManager ll,
                                    long timeoutMs,
                                    double areaThreshold,
                                    double txDeadbandDeg,
                                    double turnGain,
                                    double drivePower) {
        intake.setPower(IntakeConfig.INTAKE_FWD_POWER);
        long deadline = System.currentTimeMillis() + timeoutMs;

        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            if (System.currentTimeMillis() >= deadline) {
                intake.setPower(0);
                return false;
            }

            Double tx   = ll.getBlobTx();
            double area = ll.getBlobArea();

            if (area >= areaThreshold) {
                intake.setPower(0);
                return true;
            }

            if (tx == null) {
                follower.setTeleOpDrive(0, 0, 0, true);
            } else {
                double turn    = -tx * turnGain;
                double forward = (Math.abs(tx) <= txDeadbandDeg) ? drivePower : 0;
                follower.setTeleOpDrive(forward, 0, turn, true);
            }
            follower.update();
        }

        intake.setPower(0);
        return false;
    }

    /**
     * Vision-guided ball chase with default tuning values from {@link AutoFSMConfig}.
     */
    public static boolean chaseBlob(LinearOpMode opMode,
                                    IntakeSubsystem intake,
                                    Follower follower,
                                    LimelightManager ll,
                                    long timeoutMs,
                                    double areaThreshold) {
        return chaseBlob(opMode, intake, follower, ll,
                timeoutMs, areaThreshold,
                AutoFSMConfig.INTAKE_TX_DEADBAND_DEG,
                AutoFSMConfig.INTAKE_TURN_GAIN,
                AutoFSMConfig.INTAKE_DRIVE_POWER);
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // PATH FOLLOWING
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /**
     * Block until the PedroAutoRunner finishes its current path (or times out).
     * Calls {@code runner.update()} and {@code shooter.updatePID()} every iteration
     * so both the follower and flywheel PID stay alive.
     *
     * @param timeoutMs give up after this many ms (0 = no timeout)
     * @return true if path completed normally, false if timed out or stopped
     */
    public static boolean waitForPath(LinearOpMode opMode,
                                      PedroAutoRunner runner,
                                      ShooterSubsystem shooter,
                                      long timeoutMs) {
        long deadline = (timeoutMs > 0) ? System.currentTimeMillis() + timeoutMs : Long.MAX_VALUE;
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && runner.isBusy()) {
            runner.update();
            if (shooter != null) shooter.updatePID();
            if (System.currentTimeMillis() >= deadline) return false;
        }
        return !opMode.isStopRequested();
    }

    /**
     * Wait for path without a shooter (no flywheel PID to maintain).
     */
    public static boolean waitForPath(LinearOpMode opMode,
                                      PedroAutoRunner runner,
                                      long timeoutMs) {
        return waitForPath(opMode, runner, null, timeoutMs);
    }

    /**
     * Wait for path while also running the intake (e.g. extending intake during
     * transit to a pickup zone to save time).
     */
    public static boolean waitForPathWithIntake(LinearOpMode opMode,
                                                PedroAutoRunner runner,
                                                ShooterSubsystem shooter,
                                                IntakeSubsystem intake,
                                                long timeoutMs) {
        intake.setPower(IntakeConfig.INTAKE_FWD_POWER);
        boolean result = waitForPath(opMode, runner, shooter, timeoutMs);
        intake.setPower(0);
        return result;
    }

    /**
     * Wait for path while pre-spinning the flywheel to {@code targetRpm}.
     * The shooter is left at speed when this returns so you can fire immediately.
     * Returns true if path completed before timeout.
     */
    public static boolean waitForPathSpinning(LinearOpMode opMode,
                                              PedroAutoRunner runner,
                                              ShooterSubsystem shooter,
                                              double targetRpm,
                                              long timeoutMs) {
        shooter.setShooterVelocityRpm(targetRpm);
        return waitForPath(opMode, runner, shooter, timeoutMs);
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // TURRET / AIM
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /**
     * Block until the turret's auto-aim centres on the given AprilTag
     * (TX within {@code toleranceDeg} degrees), or until timeout.
     * Runs the flywheel PID and calls {@code shooter.runTurretControl()} every loop.
     *
     * @param tagId        the AprilTag ID to aim at
     * @param toleranceDeg acceptable TX error (degrees)
     * @param timeoutMs    give up after this many ms
     * @return true if the turret reached the target, false if timed out
     */
    public static boolean aimTurret(LinearOpMode opMode,
                                    ShooterSubsystem shooter,
                                    LimelightManager ll,
                                    int tagId,
                                    double toleranceDeg,
                                    long timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            shooter.runTurretControl(0, false);
            shooter.updatePID();
            Double tx = ll.getTagTx(tagId);
            if (tx != null && Math.abs(tx) <= toleranceDeg) return true;
            if (System.currentTimeMillis() >= deadline) return false;
        }
        return false;
    }

    /**
     * Aim the turret using {@link ShooterConfig#AUTO_AIM_DEADBAND_DEG} as tolerance.
     */
    public static boolean aimTurret(LinearOpMode opMode,
                                    ShooterSubsystem shooter,
                                    LimelightManager ll,
                                    int tagId,
                                    long timeoutMs) {
        return aimTurret(opMode, shooter, ll, tagId,
                ShooterConfig.AUTO_AIM_DEADBAND_DEG, timeoutMs);
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // HOOD
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /**
     * Set the hood to the calibrated angle for a given camera-to-target distance.
     * Uses the cubic polynomial from {@link ShooterConfig#hoodPitch}.
     *
     * @param distanceCm planar camera-to-target distance in centimetres
     */
    public static void setHoodForDistance(HoodSubsystem hood, double distanceCm) {
        hood.setPosition(ShooterConfig.hoodPitch(distanceCm));
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // TIMING UTILITIES
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /**
     * Sleep for {@code ms} milliseconds while keeping the flywheel PID alive.
     * Use instead of {@code opMode.sleep()} whenever the shooter is spinning,
     * because {@code opMode.sleep()} blocks without calling {@code updatePID()},
     * which causes the flywheel to decelerate.
     */
    public static void sleepWithPid(LinearOpMode opMode,
                                    ShooterSubsystem shooter,
                                    long ms) {
        long deadline = System.currentTimeMillis() + ms;
        while (opMode.opModeIsActive() && !opMode.isStopRequested()
                && System.currentTimeMillis() < deadline) {
            if (shooter != null) shooter.updatePID();
        }
    }

    /**
     * Sleep for {@code ms} ms while running both the PID and path follower.
     * Useful for a brief wait at a waypoint between actions.
     */
    public static void sleepWithPidAndFollower(LinearOpMode opMode,
                                               ShooterSubsystem shooter,
                                               PedroAutoRunner runner,
                                               long ms) {
        long deadline = System.currentTimeMillis() + ms;
        while (opMode.opModeIsActive() && !opMode.isStopRequested()
                && System.currentTimeMillis() < deadline) {
            if (shooter != null) shooter.updatePID();
            if (runner  != null) runner.update();
        }
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // DEFAULT TUNING (inner class — change here to affect all chaseBlob calls)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    public static final class AutoFSMConfig {
        public static double INTAKE_TX_DEADBAND_DEG = 6.0;
        public static double INTAKE_TURN_GAIN       = 0.025;
        public static double INTAKE_DRIVE_POWER     = 0.30;
    }
}
