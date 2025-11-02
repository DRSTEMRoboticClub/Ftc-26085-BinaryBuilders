package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.Servo;

public class CameraController {
    private final Servo cameraServo;

    public CameraController(Servo servo) {
        cameraServo = servo;
    }

    public void Down() {
        cameraServo.setPosition(0.05);
    }

    public void Up() {
        cameraServo.setPosition(0.15);
    }

    public void Front() {
        cameraServo.setPosition(0.95);
    }
}
