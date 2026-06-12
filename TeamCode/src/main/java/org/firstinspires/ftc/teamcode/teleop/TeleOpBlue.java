package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.teleop.subsystems.*;
import org.firstinspires.ftc.teamcode.tools.PriorityInputHandler;

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
    
    // Performance and Health Trackers
    private long lastLoopTime = 0;
    private double minVoltage = 14.0;

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
        
        // STABILITY FIX: Enable Manual Bulk Caching for all Hubs and Disable Identify Mode
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.visuallyIdentify(false); // Ensure hubs are not in "Identify" (flashing blue) mode
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // CRITICAL: Clear cache at start of loop for maximum I2C efficiency
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            long currentTime = System.currentTimeMillis();
            long loopTime = (lastLoopTime == 0) ? 0 : currentTime - lastLoopTime;
            lastLoopTime = currentTime;

            // Update Logic
            inputHandler.update(drive, intake, shooter, hood);

            // --- HEALTH MONITORING ---
            double voltage = batteryVoltageSensor.getVoltage();
            if (voltage < minVoltage) minVoltage = voltage;
            
            // Analyze Disconnection Risks
            String voltStatus = (voltage < 11.0) ? "!! BROWNOUT RISK !!" : (voltage < 12.0) ? "! LOW !" : "OK";
            String loopStatus = (loopTime > 45) ? "!! HIGH LATENCY !!" : "OK";

            telemetry.addLine("=== DISCONNECT MONITOR ===");
            telemetry.addData("Battery", "%.2fV [%s]", voltage, voltStatus);
            telemetry.addData("Min Voltage Seen", "%.2fV (Goal: >11V)", minVoltage);
            telemetry.addData("Loop Time", "%d ms [%s]", loopTime, loopStatus);
            
            telemetry.addLine("=== CONTROL STATUS ===");
            telemetry.addData("G1/G2 Priority", "%s | %s", 
                inputHandler.isG1Priority() ? "G1" : "G2", 
                inputHandler.isG1Active() ? "G1 Active" : "G2 Active");
            
            telemetry.addLine("=== DRIVEBASE ===");
            telemetry.addData("Heading", "%.1f°", drive.getHeading());
            telemetry.addData("PID Target", "%.1f°", drive.getTargetHeading());
            telemetry.addData("Joy Sticks", "F:%.2f S:%.2f T:%.2f",
                inputHandler.getForward(), inputHandler.getStrafe(), inputHandler.getTurn());
            telemetry.addData("G1 L-Stick Y", "%.3f", gamepad1.left_stick_y);
            telemetry.addData("G1 L-Stick X", "%.3f", gamepad1.left_stick_x);
            telemetry.addData("G1 R-Stick X", "%.3f", gamepad1.right_stick_x);
            telemetry.addData("G1 Active", inputHandler.isG1Active());
            telemetry.addData("G2 Active", inputHandler.isG2Active());

            // Error Detection
            double totalInput = Math.abs(inputHandler.getForward()) + Math.abs(inputHandler.getStrafe()) + Math.abs(inputHandler.getTurn());
            if (totalInput > 0.01 && (Math.abs(gamepad1.left_stick_y) < 0.01 && Math.abs(gamepad1.left_stick_x) < 0.01 && Math.abs(gamepad1.right_stick_x) < 0.01)) {
                telemetry.addData("ERROR", "Input persisting after joystick release!");
            }
            
            telemetry.addLine("=== SUBSYSTEMS ===");
            telemetry.addData("Shooter", inputHandler.isManualMode() ? "MANUAL" : "AUTO-AIM");
            telemetry.addData("Launcher Power", "%.2f / Max %.2f",
                shooter.getShooterPower(), org.firstinspires.ftc.teamcode.configs.ShooterConfig.MAX_LAUNCHER_POWER);
            telemetry.addData("Hood Angle", "%.3f", hood.getPosition());

            telemetry.update();
        }

        shooter.stopLimelight();
    }
}
