package org.firstinspires.ftc.teamcode.tools;

public class MathUtils {
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    
    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
