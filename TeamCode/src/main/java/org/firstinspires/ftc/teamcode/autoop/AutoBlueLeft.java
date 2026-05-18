package org.firstinspires.ftc.teamcode.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
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

        runner.follow(scorePath);
        
        while (opModeIsActive() && runner.isBusy()) {
            runner.update();
            // Parallel: Spin up shooter while driving
            shooter.setShooterPower(0.8);
        }

        // Fire
        shooter.setStopperPosition(1.0);
        sleep(500);
        shooter.setShooterPower(0);
        
        shooter.stopLimelight();
    }
}
