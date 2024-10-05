package org.firstinspires.ftc.teamcode.tools;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class ControlPad {
    private Gamepad gamepad;
    private Telemetry telemetry;

    private boolean left_bumper_ready = true;
    private boolean right_bumper_ready = true;

    public ControlPad(Telemetry the_telemetry, Gamepad the_gamepad)
    {
        gamepad = the_gamepad;
        telemetry = the_telemetry;
    }

    public boolean is_left_bumper_pressed()
    {
        if (left_bumper_ready)
        {
            if (gamepad.left_bumper)
            {
                left_bumper_ready = false;
                return true;
            }
        }
        else
        {
            if (!gamepad.left_bumper)
            {
                left_bumper_ready = true;
            }
        }
        return false;
    }

    public boolean is_right_bumper_pressed()
    {
        if (right_bumper_ready)
        {
            if (gamepad.right_bumper)
            {
                right_bumper_ready = false;
                return true;
            }
        }
        else
        {
            if (!gamepad.right_bumper)
            {
                right_bumper_ready = true;
            }
        }
        return false;
    }
}
