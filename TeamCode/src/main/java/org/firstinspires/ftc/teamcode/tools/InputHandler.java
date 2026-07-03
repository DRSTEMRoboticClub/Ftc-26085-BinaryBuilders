package org.firstinspires.ftc.teamcode.tools;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.configs.*;
import org.firstinspires.ftc.teamcode.teleop.subsystems.*;

import java.util.HashMap;
import java.util.Map;

public class InputHandler {
    private final GamepadEx g1, g2;
    private boolean fieldCentric = true;       // G1 Y = field centric (default), G1 X = robot centric
    private boolean shooterHoldMode = false;   // true while G2 left trigger is held (autoaim + spin-up)
    private final Map<String, Long> nextRepeatTimesMs = new HashMap<>();

    private double lastForward, lastStrafe, lastTurn;
    private long shootingStartMs = 0;
    private boolean wasShootingPrev = false;

    public InputHandler(GamepadEx g1, GamepadEx g2) {
        this.g1 = g1;
        this.g2 = g2;
    }

    /** True when NOT in autoaim mode (G2 left trigger not held) — hood uses manual joystick. */
    public boolean isManualMode() {
        return !shooterHoldMode;
    }

    /** True while G2 left trigger is held (autoaim + polynomial spin-up active). */
    public boolean isShooterHoldMode() {
        return shooterHoldMode;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public double getForward() { return lastForward; }
    public double getStrafe() { return lastStrafe; }
    public double getTurn() { return lastTurn; }

    public void update(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, HoodSubsystem hood) {
        g1.readButtons();
        g2.readButtons();

        // Emergency: G1 A — reverse intake, reverse launcher, open stopper
        if (g1.gamepad.a) {
            intake.setPower(IntakeConfig.INTAKE_REV_POWER);
            shooter.setShooterVelocityRpm(-0.5 * ShooterConfig.MAX_LAUNCHER_RPM);
            shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
            shooter.setAutoAimEnabled(false);
            shooter.updatePID();
            return;
        }

        // ── Controller 1: Drive ──────────────────────────────────────────────

        // Left Bumper: snap to 0° heading — held continuously so PID corrects until released
        if (g1.gamepad.left_bumper) {
            drive.setTargetHeading(0);
        }

        // Right Bumper: slow mode while held
        drive.setSlowMode(g1.gamepad.right_bumper);

        // X: robot centric  |  Y: field centric
        if (g1.wasJustPressed(GamepadKeys.Button.X)) fieldCentric = false;
        if (g1.wasJustPressed(GamepadKeys.Button.Y)) fieldCentric = true;

        // B: park (disable drive while held)
        if (g1.gamepad.b) {
            drive.stop();
            lastStrafe = 0; lastForward = 0; lastTurn = 0;
        } else {
            lastStrafe  = -g1.getLeftX();
            lastForward = -g1.getLeftY();
            // Suppress manual turn while snapping to heading so PID has full authority
            lastTurn    = g1.gamepad.left_bumper ? 0 : -g1.getRightX();
            if (fieldCentric) {
                drive.driveFieldCentric(lastStrafe, lastForward, lastTurn);
            } else {
                drive.driveRobotCentric(lastStrafe, lastForward, lastTurn);
            }
        }

        // Left Trigger: intake while held
        boolean intaking = g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > ControlsConfig.TRIGGER_THRESHOLD;

        // ── Controller 2: Shooter / Turret / Hood ───────────────────────────

        // Left Trigger: autoaim + start launcher while held (polynomial RPM via applyShooterCompensation)
        shooterHoldMode = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > ControlsConfig.TRIGGER_THRESHOLD;
        shooter.setAutoAimEnabled(shooterHoldMode);

        // Right Joystick X: turret manual
        double turretManual = -g2.getRightX();
        shooter.runTurretControl(
                Math.abs(turretManual) > ControlsConfig.G2_DEADZONE ? turretManual : 0,
                shooterHoldMode);

        // Left Joystick Y: hood up/down proportional (auto-hood via polynomial when autoaim active)
        double hoodInput = g2.getLeftY();
        if (Math.abs(hoodInput) > ControlsConfig.G2_DEADZONE) {
            hood.adjustPosition(hoodInput * HoodConfig.HOOD_MANUAL_RATE);
        }

        // Launcher RPM:
        //   A held (neutral/calibration) → CALIBRATION_RPM, hood untouched, autoaim off
        //   Left Trigger held (autoaim)  → polynomial override set by applyShooterCompensation;
        //                                  setShooterVelocityRpm is ignored while override is active,
        //                                  but we set NEAR_RPM here as a fallback when no tag is visible
        //   Left Bumper held             → near RPM (manual, no autoaim)
        //   Right Bumper held            → far RPM  (manual, no autoaim)
        //   Otherwise                    → stop
        boolean neutralShot = g2.gamepad.a;
        if (neutralShot) {
            shooter.setAutoAimEnabled(false);
            shooter.setShooterVelocityRpm(ShooterConfig.CALIBRATION_RPM);
            // hood is intentionally not touched — stays at whatever position it was set to
        } else if (shooterHoldMode) {
            shooter.setShooterVelocityRpm(ShooterConfig.NEAR_RPM); // ignored if polynomial override is active
        } else if (g2.gamepad.left_bumper) {
            shooter.setShooterVelocityRpm(ShooterConfig.NEAR_RPM);
            hood.setPosition(ShooterConfig.NEAR_PITCH);
        } else if (g2.gamepad.right_bumper) {
            shooter.setShooterVelocityRpm(ShooterConfig.FAR_RPM);
            hood.setPosition(ShooterConfig.FAR_PITCH);
        } else {
            shooter.setShooterVelocityRpm(0);
        }
        shooter.updatePID();

        long now = System.currentTimeMillis();

        // Right Trigger: shoot (open stopper + feed intake); takes priority over plain intake
        boolean shooting = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > ControlsConfig.TRIGGER_THRESHOLD;
        if (shooting) {
            if (!wasShootingPrev) shootingStartMs = now;
            shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
            boolean intakeReady = (now - shootingStartMs) >= ShootZoneConfig.SHOT_INTAKE_DELAY_MS;
            intake.setPower(intakeReady ? IntakeConfig.INTAKE_FWD_POWER : 0);
        } else {
            shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
            intake.setPower(intaking ? IntakeConfig.INTAKE_FWD_POWER : 0);
        }
        wasShootingPrev = shooting;

        // ── G2 Y/X: calibration RPM tuning (used with A neutral shot) ───────────
        // Y = raise CALIBRATION_RPM, X = lower it
        if (shouldStep("g2_y", g2.gamepad.y, g2.wasJustPressed(GamepadKeys.Button.Y), now)) {
            ShooterConfig.CALIBRATION_RPM = Math.min(ShooterConfig.MAX_LAUNCHER_RPM,
                    ShooterConfig.CALIBRATION_RPM + ShooterConfig.RPM_TUNE_STEP_COARSE);
        } else if (shouldStep("g2_x", g2.gamepad.x, g2.wasJustPressed(GamepadKeys.Button.X), now)) {
            ShooterConfig.CALIBRATION_RPM = Math.max(0.0,
                    ShooterConfig.CALIBRATION_RPM - ShooterConfig.RPM_TUNE_STEP_COARSE);
        }

        // ── G2 D-pad: live RPM tuning ────────────────────────────────────────
        // Up/Down: adjust near RPM  |  Left/Right: adjust far RPM
        if (shouldStep("g2_dpad_up", g2.gamepad.dpad_up,
                g2.wasJustPressed(GamepadKeys.Button.DPAD_UP), now)) {
            ShooterConfig.NEAR_RPM = Math.min(ShooterConfig.MAX_LAUNCHER_RPM,
                    ShooterConfig.NEAR_RPM + ShooterConfig.RPM_TUNE_STEP_COARSE);
        } else if (shouldStep("g2_dpad_down", g2.gamepad.dpad_down,
                g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN), now)) {
            ShooterConfig.NEAR_RPM = Math.max(0.0,
                    ShooterConfig.NEAR_RPM - ShooterConfig.RPM_TUNE_STEP_COARSE);
        }
        if (shouldStep("g2_dpad_right", g2.gamepad.dpad_right,
                g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT), now)) {
            ShooterConfig.FAR_RPM = Math.min(ShooterConfig.MAX_LAUNCHER_RPM,
                    ShooterConfig.FAR_RPM + ShooterConfig.RPM_TUNE_STEP_COARSE);
        } else if (shouldStep("g2_dpad_left", g2.gamepad.dpad_left,
                g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT), now)) {
            ShooterConfig.FAR_RPM = Math.max(0.0,
                    ShooterConfig.FAR_RPM - ShooterConfig.RPM_TUNE_STEP_COARSE);
        }
    }

    private boolean shouldStep(String key, boolean pressed, boolean justPressed, long nowMs) {
        if (justPressed) {
            nextRepeatTimesMs.put(key, nowMs + ShooterConfig.TUNE_INITIAL_REPEAT_MS);
            return true;
        }
        if (!pressed) {
            nextRepeatTimesMs.remove(key);
            return false;
        }
        long next = nextRepeatTimesMs.getOrDefault(key, nowMs + ShooterConfig.TUNE_INITIAL_REPEAT_MS);
        if (nowMs >= next) {
            nextRepeatTimesMs.put(key, nowMs + ShooterConfig.TUNE_REPEAT_MS);
            return true;
        }
        return false;
    }
}
