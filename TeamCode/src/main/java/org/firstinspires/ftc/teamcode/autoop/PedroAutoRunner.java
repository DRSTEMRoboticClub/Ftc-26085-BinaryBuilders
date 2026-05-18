package org.firstinspires.ftc.teamcode.autoop;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.configs.DriveConfig;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;

public class PedroAutoRunner {
    private final Follower follower;

    public PedroAutoRunner(HardwareMap hMap) {
        MecanumConstants mConstants = new MecanumConstants();
        mConstants.leftFrontMotorName = HardwareConfig.FL_NAME;
        mConstants.rightFrontMotorName = HardwareConfig.FR_NAME;
        mConstants.leftRearMotorName = HardwareConfig.BL_NAME;
        mConstants.rightRearMotorName = HardwareConfig.BR_NAME;
        mConstants.nominalVoltage = DriveConfig.NOMINAL_VOLTAGE;

        DriveEncoderConstants lConstants = new DriveEncoderConstants();
        lConstants.leftFrontMotorName = HardwareConfig.FL_NAME;
        lConstants.rightFrontMotorName = HardwareConfig.FR_NAME;
        lConstants.leftRearMotorName = HardwareConfig.BL_NAME;
        lConstants.rightRearMotorName = HardwareConfig.BR_NAME;

        follower = new FollowerBuilder(new FollowerConstants(), hMap)
                .mecanumDrivetrain(mConstants)
                .driveEncoderLocalizer(lConstants)
                .build();
    }

    public void follow(Path path) {
        follower.followPath(path);
    }

    public void follow(PathChain pathChain) {
        follower.followPath(pathChain);
    }

    public void update() {
        follower.update();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }
}
