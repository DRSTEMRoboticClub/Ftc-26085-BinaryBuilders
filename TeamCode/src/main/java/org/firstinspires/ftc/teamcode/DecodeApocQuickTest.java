package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "Decode APOC Quick Test", group = "Test")
public class DecodeApocQuickTest extends LinearOpMode {

    // Hardware names from HWConfig.docx
    private static final String FL_NAME = "FLWheelMotor";
    private static final String FR_NAME = "FRWheelMotor";
    private static final String BL_NAME = "BLWheelMotor";
    private static final String BR_NAME = "BRWheelMotor";
    private static final String INTAKE_NAME = "IntakeMotor";
    private static final String TURRET_NAME = "TurretMotor";
    private static final String LIMELIGHT_NAME = "LimeCam";
    private static final String IMU_NAME = "imu";

    // Turret gearing: REV motor (28 ticks/rev) * 45:1 gearbox * (72/16) external ratio.
    private static final double TURRET_TICKS_PER_DEGREE = (28.0 * 45.0 * (72.0 / 16.0)) / 360.0;
    private static final double TURRET_MIN_DEG = -180.0;
    private static final double TURRET_MAX_DEG = 180.0;

    // Drive tuning
    private static final double DRIVE_SCALE = 1.0;
    private static final double TURN_SCALE = 0.8;

    // Intake tuning
    private static final double INTAKE_FWD_POWER = 0.9;
    private static final double INTAKE_REV_POWER = -0.8;

    // Turret control tuning
    private static final double TURRET_MANUAL_POWER = 0.45;
    private static final double TURRET_KP = 0.018;
    private static final double TURRET_KI = 0.0;
    private static final double TURRET_KD = 0.0012;
    private static final double TURRET_MAX_AUTO_POWER = 0.45;
    private static final double AIM_DEADBAND_DEG = 0.35;

    // Pipeline index used on Limelight for AprilTag targeting.
    private static final int APRILTAG_PIPELINE = 0;
    private static final int TRACKED_TAG_ID = 20;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotorEx turret;
    private IMU imu;
    private Limelight3A limelight;

    private boolean autoAimEnabled = false;
    private boolean xPressedLast = false;

    private double turretTargetDeg = 0.0;
    private double turretIntegral = 0.0;
    private double turretPrevError = 0.0;
    private boolean trackedTagVisible = false;
    private double trackedTagTxDeg = 0.0;

    private final ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Decode APOC quick test ready");
        telemetry.addLine("G1: left stick drive, right stick turn, A reset yaw");
        telemetry.addLine("G2: RT intake in, LT intake out");
        telemetry.addLine("G2: right stick turret manual");
        telemetry.addLine("G2: X toggle auto-aim with Limelight tx");
        telemetry.addLine("Turret zero note: face launcher forward before INIT if possible");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {
            driveFieldCentric();
            runIntake();
            runTurretControl();
            sendTelemetry();
        }

        if (limelight != null) {
            limelight.stop();
        }
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, FL_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FR_NAME);
        backLeft = hardwareMap.get(DcMotor.class, BL_NAME);
        backRight = hardwareMap.get(DcMotor.class, BR_NAME);
        intake = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        turret = hardwareMap.get(DcMotorEx.class, TURRET_NAME);

        // Adjust these directions if any wheel spins opposite during testing.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.FORWARD);

        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();

        try {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
            telemetry.addLine("WARNING: Limelight not found as 'LimeCam'");
        }
    }

    private void driveFieldCentric() {
        if (gamepad1.a) {
            imu.resetYaw();
        }

        double forward = -gamepad1.left_stick_y * DRIVE_SCALE;
        double strafe = gamepad1.left_stick_x * DRIVE_SCALE;
        double turn = gamepad1.right_stick_x * TURN_SCALE;

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double theta = Math.atan2(forward, strafe);
        double magnitude = Math.hypot(forward, strafe);

        theta = AngleUnit.normalizeRadians(theta - yaw);

        double robotForward = magnitude * Math.sin(theta);
        double robotStrafe = magnitude * Math.cos(theta);

        double fl = robotForward + robotStrafe + turn;
        double fr = robotForward - robotStrafe - turn;
        double bl = robotForward - robotStrafe + turn;
        double br = robotForward + robotStrafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    private void runIntake() {
        double intakePower = 0.0;
        if (gamepad2.right_trigger > 0.1) {
            intakePower = INTAKE_FWD_POWER;
        } else if (gamepad2.left_trigger > 0.1) {
            intakePower = INTAKE_REV_POWER;
        }
        intake.setPower(intakePower);
    }

    private void runTurretControl() {
        boolean xPressed = gamepad2.x;
        if (xPressed && !xPressedLast) {
            autoAimEnabled = !autoAimEnabled;
            turretIntegral = 0.0;
            turretPrevError = 0.0;
        }
        xPressedLast = xPressed;

        double manualInput = -gamepad2.right_stick_x;
        if (Math.abs(manualInput) > 0.08) {
            autoAimEnabled = false;
            turret.setPower(manualInput * TURRET_MANUAL_POWER);
            turretTargetDeg = getTurretAngleDeg();
            return;
        }

        if (!autoAimEnabled || limelight == null) {
            turret.setPower(0.0);
            turretTargetDeg = getTurretAngleDeg();
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            trackedTagVisible = false;
            turret.setPower(0.0);
            return;
        }

        Double tx = getTrackedTagTx(result, TRACKED_TAG_ID);
        if (tx == null) {
            trackedTagVisible = false;
            turret.setPower(0.0);
            return;
        }

        trackedTagVisible = true;
        trackedTagTxDeg = tx;
        turretTargetDeg = clamp(getTurretAngleDeg() - tx, TURRET_MIN_DEG, TURRET_MAX_DEG);
        double error = turretTargetDeg - getTurretAngleDeg();

        if (Math.abs(error) < AIM_DEADBAND_DEG) {
            turret.setPower(0.0);
            turretIntegral = 0.0;
            turretPrevError = error;
            return;
        }

        double dt = Math.max(0.001, pidTimer.seconds());
        pidTimer.reset();

        turretIntegral += error * dt;
        turretIntegral = Range.clip(turretIntegral, -30.0, 30.0);
        double derivative = (error - turretPrevError) / dt;
        turretPrevError = error;

        double output = (TURRET_KP * error) + (TURRET_KI * turretIntegral) + (TURRET_KD * derivative);
        output = Range.clip(output, -TURRET_MAX_AUTO_POWER, TURRET_MAX_AUTO_POWER);

        double current = getTurretAngleDeg();
        if ((current <= TURRET_MIN_DEG && output < 0.0) || (current >= TURRET_MAX_DEG && output > 0.0)) {
            output = 0.0;
        }

        turret.setPower(output);
    }

    private Double getTrackedTagTx(LLResult result, int tagId) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) {
            return null;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == tagId) {
                return fiducial.getTargetXDegrees();
            }
        }

        return null;
    }

    private double getTurretAngleDeg() {
        return turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void sendTelemetry() {
        double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addData("Drive Yaw (deg)", "%.1f", yawDeg);
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Turret Angle (deg)", "%.2f", getTurretAngleDeg());
        telemetry.addData("Turret Target (deg)", "%.2f", turretTargetDeg);
        telemetry.addData("Auto Aim", autoAimEnabled ? "ON" : "OFF");

        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("Limelight tx", "%.2f", result.getTx());
                telemetry.addData("Limelight ty", "%.2f", result.getTy());
                telemetry.addData("Tracked Tag", trackedTagVisible ? ("ID " + TRACKED_TAG_ID + " visible") : ("ID " + TRACKED_TAG_ID + " not visible"));
                if (trackedTagVisible) {
                    telemetry.addData("Tracked Tag tx", "%.2f", trackedTagTxDeg);
                } else {
                    telemetry.addData("Tracked Tag tx", "--");
                }
            } else {
                telemetry.addLine("Limelight: no valid target");
            }
        } else {
            telemetry.addLine("Limelight: not connected in hardware map");
        }

        telemetry.update();
    }
}
