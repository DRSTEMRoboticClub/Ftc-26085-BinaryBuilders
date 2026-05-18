package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConfig {
    public static double DRIVE_SCALE = 1.0;
    public static double TURN_SCALE = 0.8;
    public static double SLOW_MODE_SCALE = 0.4;
    
    // Set to 0.0 to stop erratic behavior and Hub disconnects
    public static double HEADING_P = 0.0; 
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.0;
    
    // Physical Constants
    public static double NOMINAL_VOLTAGE = 12.0;
}
