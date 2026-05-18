package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.configs.HardwareConfig;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx intake;

    public IntakeSubsystem(HardwareMap hMap) {
        intake = hMap.get(DcMotorEx.class, HardwareConfig.INTAKE_NAME);
    }

    public void setPower(double power) {
        intake.setPower(power);
    }
}
