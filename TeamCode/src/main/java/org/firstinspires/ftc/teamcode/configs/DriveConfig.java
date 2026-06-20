package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConfig {
    // Speed Scaling (0.0 to 1.0)
    public static double NORMAL_SPEED_SCALE = 1.0;
    public static double SLOW_MODE_SPEED_SCALE = 0.5;
    public static double VERY_SLOW_MODE_SCALE = 0.2;
    public static double TURN_SCALE = 0.8;

    // Joystick Deadzone (0.0 to 1.0)
    public static double JOYSTICK_DEADZONE = 0.05;

    // Heading PID — DISABLED (0) on purpose.
    // Aiming is done by the TURRET, not the chassis. With a non-zero P, the drive latches
    // targetHeading the instant the driver stops turning (e.g. while aiming at a tag) and then
    // fights the driver back to that heading — felt as the robot "drifting to the heading it saw
    // the AprilTag". At 0, driveFieldCentric skips the heading block entirely and turn input
    // passes through raw, so the chassis drives freely and the turret handles all aiming.
    public static double HEADING_P = 0.0;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.0;

    // Motor Velocity PID (for potential future use)
    public static double MOTOR_P = 0.001;
    public static double MOTOR_I = 0.0;
    public static double MOTOR_D = 0.0;

    // Physical Constants
    public static double NOMINAL_VOLTAGE = 12.0;
}
