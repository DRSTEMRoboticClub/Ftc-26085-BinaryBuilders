package org.firstinspires.ftc.teamcode.tools;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.configs.*;
import org.firstinspires.ftc.teamcode.teleop.subsystems.*;

public class PriorityInputHandler {
    private final GamepadEx g1, g2;
    private boolean g1HasPriority = true;
    private boolean manualMode = false;
    private boolean verySlowMode = false;

    private double lastForward, lastStrafe, lastTurn;

    public PriorityInputHandler(GamepadEx g1, GamepadEx g2) {
        this.g1 = g1;
        this.g2 = g2;
    }

    public boolean isG1Priority() {
        return g1HasPriority;
    }

    public boolean isManualMode() {
        return manualMode;
    }

    public double getForward() { return lastForward; }
    public double getStrafe() { return lastStrafe; }
    public double getTurn() { return lastTurn; }

    public boolean isG1Active() {
        return Math.abs(g1.getLeftY()) > ControlsConfig.G1_DEADZONE ||
               Math.abs(g1.getLeftX()) > ControlsConfig.G1_DEADZONE ||
               Math.abs(g1.getRightX()) > ControlsConfig.G1_DEADZONE ||
               g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > ControlsConfig.TRIGGER_THRESHOLD ||
               g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > ControlsConfig.TRIGGER_THRESHOLD ||
               g1.gamepad.a || g1.gamepad.b || g1.gamepad.x || g1.gamepad.y ||
               g1.gamepad.dpad_up || g1.gamepad.dpad_down || g1.gamepad.dpad_left || g1.gamepad.dpad_right ||
               g1.gamepad.left_bumper || g1.gamepad.right_bumper;
    }

    public boolean isG2Active() {
        return Math.abs(g2.getLeftY()) > ControlsConfig.G2_DEADZONE ||
               Math.abs(g2.getLeftX()) > ControlsConfig.G2_DEADZONE ||
               Math.abs(g2.getRightX()) > ControlsConfig.G2_DEADZONE ||
               g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > ControlsConfig.TRIGGER_THRESHOLD ||
               g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > ControlsConfig.TRIGGER_THRESHOLD ||
               g2.gamepad.a || g2.gamepad.b || g2.gamepad.x || g2.gamepad.y ||
               g2.gamepad.dpad_up || g2.gamepad.dpad_down || g2.gamepad.dpad_left || g2.gamepad.dpad_right ||
               g2.gamepad.left_bumper || g2.gamepad.right_bumper;
    }

    public void update(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, HoodSubsystem hood) {
        g1.readButtons();
        g2.readButtons();

        // Reset IMU Heading (G1 B)
        if (g1.wasJustPressed(GamepadKeys.Button.B)) {
            drive.resetHeading();
        }

        // Emergency Logic (A button on either)
        if (g1.gamepad.a || g2.gamepad.a) {
            intake.setPower(IntakeConfig.INTAKE_REV_POWER);
            shooter.setShooterPower(-0.5);
            shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
            return;
        }

        // Determine Active Controller for Movement
        GamepadEx moveController;
        if (g1HasPriority) {
            moveController = isG1Active() ? g1 : g2;
        } else {
            moveController = isG2Active() ? g2 : g1;
        }

        // Slow Mode (G1 Right Bumper)
        drive.setSlowMode(g1.getButton(GamepadKeys.Button.RIGHT_BUMPER));

        // Very Slow Mode (G2 Left Button)
        if (g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            verySlowMode = !verySlowMode;
        }
        drive.setVerySlow(verySlowMode);

        // Drivetrain (Field Centric)
        // Left Stick: Move, Right Stick X: Turn
        lastStrafe = -moveController.getLeftX();
        lastForward = -moveController.getLeftY();
        lastTurn = -moveController.getRightX();
        drive.driveFieldCentric(lastStrafe, lastForward, lastTurn);

        // Shooter Logic (G1 Primary)
        // Right Trigger - Ramp up launcher speed target (velocity control)
        double rightTriggerPower = g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        shooter.setShooterPower(rightTriggerPower);

        // Left Bumper - Shoot (Open Stopper + Auto Intake)
        if (g1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
            intake.setPower(IntakeConfig.INTAKE_FWD_POWER);
        } else {
            shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);

            // Intake Logic (G1 Left Trigger) when not shooting
            double intakePower = 0;
            if (g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > ControlsConfig.TRIGGER_THRESHOLD) {
                intakePower = IntakeConfig.INTAKE_FWD_POWER;
            }
            intake.setPower(intakePower);
        }

        // Controller 2 Intake/Outtake
        // Right Trigger - Intake, Left Trigger - Outtake
        double g2IntakePower = 0;
        if (g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > ControlsConfig.TRIGGER_THRESHOLD) {
            g2IntakePower = IntakeConfig.INTAKE_FWD_POWER;
        } else if (g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > ControlsConfig.TRIGGER_THRESHOLD) {
            g2IntakePower = IntakeConfig.INTAKE_REV_POWER;
        }

        // G2 intake takes priority over G1 when G2 is actively using it
        if (Math.abs(g2IntakePower) > 0.1) {
            intake.setPower(g2IntakePower);
        }

        // Toggle Manual/Auto Tracking (G1 X)
        if (g1.wasJustPressed(GamepadKeys.Button.X)) {
            manualMode = !manualMode;
            shooter.toggleAutoAim();
        }

        // Hood Control (G1 D-Pad Up/Down) - Always available
        if (g1.gamepad.dpad_up) {
            hood.adjustPosition(-HoodConfig.HOOD_INCREMENT);
        } else if (g1.gamepad.dpad_down) {
            hood.adjustPosition(HoodConfig.HOOD_INCREMENT);
        }

        // Turret Control (Only if manual is ON)
        double turretManual = 0;
        if (manualMode) {
            if (g1.gamepad.dpad_left) turretManual = -1.0;
            else if (g1.gamepad.dpad_right) turretManual = 1.0;
        }
        // runTurretControl handles auto-aim logic internally
        shooter.runTurretControl(turretManual, rightTriggerPower > 0.1);

        // G2 Launcher Max Command Tuning (D-Pad Up/Down)
        // Scales trigger input before converting to velocity target
        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            ShooterConfig.MAX_LAUNCHER_POWER = Math.min(1.0,
                ShooterConfig.MAX_LAUNCHER_POWER + ShooterConfig.LAUNCHER_POWER_INCREMENT);
        } else if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            ShooterConfig.MAX_LAUNCHER_POWER = Math.max(0.0,
                ShooterConfig.MAX_LAUNCHER_POWER - ShooterConfig.LAUNCHER_POWER_INCREMENT);
        }
    }
}
