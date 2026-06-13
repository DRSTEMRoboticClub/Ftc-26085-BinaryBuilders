package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.LocalizationConfig;

/**
 * Red-alliance localization test TeleOp.
 * Drives identically to TeleOp Blue; tracks AprilTag ID 24 (family 36h11)
 * and corrects the Road Runner pose from it. All shared logic lives in
 * {@link LocalSysBase}; only the alliance constants change.
 */
@TeleOp(name = "LocalSys Red", group = "Localization")
public class LocalSysRed extends LocalSysBase {
    @Override protected int getTagId()        { return LocalizationConfig.RED_TAG_ID; }
    @Override protected double getTagFieldX()  { return LocalizationConfig.RED_TAG_FIELD_X; }
    @Override protected double getTagFieldY()  { return LocalizationConfig.RED_TAG_FIELD_Y; }
    @Override protected String getAllianceName() { return "RED"; }
}
