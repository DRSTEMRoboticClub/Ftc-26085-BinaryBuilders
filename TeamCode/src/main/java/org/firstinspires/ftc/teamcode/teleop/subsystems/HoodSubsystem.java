package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;
import org.firstinspires.ftc.teamcode.configs.HoodConfig;

public class HoodSubsystem extends SubsystemBase {
    private final Servo hood;
    private double position = 0.5;

    public HoodSubsystem(HardwareMap hMap) {
        hood = hMap.get(Servo.class, HardwareConfig.HOOD_NAME);
    }

    public void adjustPosition(double delta) {
        position = Range.clip(position + delta, HoodConfig.HOOD_MIN, HoodConfig.HOOD_MAX);
        hood.setPosition(position);
    }

    public double getPosition() {
        return position;
    }
}
