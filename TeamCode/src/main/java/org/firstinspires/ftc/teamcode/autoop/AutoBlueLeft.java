package org.firstinspires.ftc.teamcode.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.teleop.subsystems.ShooterSubsystem;

@Autonomous(name = "Blue Left Auto", group = "Production")
public class AutoBlueLeft extends LinearOpMode {
    private PedroAutoRunner runner;
    private ShooterSubsystem shooter;

    @Override
    public void runOpMode() {
        runner = new PedroAutoRunner(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        Path scorePath = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(24, 0, 0)));

        waitForStart();

        runner.followPath(scorePath);

        while (opModeIsActive() && runner.isBusy()) {
            runner.update();
            shooter.setShooterVelocityRpm(ShooterConfig.MANUAL_TARGET_RPM);
            shooter.updatePID(); // must run every loop or flywheel doesn't move
        }

        // Fire
        shooter.setStopperPosition(ShooterConfig.STOPPER_OPEN);
        long fireStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - fireStart < 500) {
            shooter.updatePID(); // keep flywheel speed during fire
        }
        shooter.setStopperPosition(ShooterConfig.STOPPER_CLOSED);
        shooter.setShooterVelocityRpm(0);

        shooter.stopLimelight();
    }
}
