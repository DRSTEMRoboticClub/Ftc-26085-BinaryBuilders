package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HoodConfig {
    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = 1.0;
    public static double HOOD_INCREMENT = 0.03;
    public static double HOOD_FINE_INCREMENT = 0.002;
    // Per-loop servo travel at full G2 right-stick deflection (manual hood-pitch control).
    public static double HOOD_MANUAL_RATE = 0.01;
}
