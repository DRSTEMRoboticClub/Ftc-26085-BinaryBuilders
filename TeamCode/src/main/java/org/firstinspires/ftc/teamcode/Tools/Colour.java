package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class Colour {
    private final String m_name;
    private final float m_red;
    private final float m_green;
    private final float m_blue;
    private final float m_alpha;

    public Colour(String name, int red, int green, int blue, int alpha)
    {
        m_name = name;
        m_red = (float)red / alpha;
        m_green = (float)green / alpha;
        m_blue = (float)blue / alpha;
        m_alpha = 1.f;
    }

    public Colour(ColorSensor c)
    {
        m_name = "Unknown";
        m_red = (float)c.red() / c.alpha();
        m_green = (float)c.green() / c.alpha();
        m_blue = (float)c.blue() / c.alpha();
        m_alpha = 1.f;
    }

    public float get_red() {
        return m_red;
    }

    public float get_green() {
        return m_green;
    }

    public float get_blue() {
        return m_blue;
    }

    public float get_alpha() {
        return m_alpha;
    }

    public String get_name() {
        return m_name;
    }

    public double get_difference(float red, float green, float blue, float alpha) {
        return Math.max((red - m_red)*(red - m_red), (red - m_red - alpha + m_alpha)*(red - m_red - alpha + m_alpha))
                + Math.max((green - m_green)*(green - m_green), (green - m_green - alpha + m_alpha)*(green - m_green - alpha + m_alpha))
                + Math.max((blue - m_blue)*(blue - m_blue), (blue - m_blue - alpha + m_alpha)*(blue - m_blue - alpha + m_alpha));
    }

    public double get_difference(Colour otherColour) {
        return get_difference(otherColour.get_red(), otherColour.get_green(), otherColour.get_blue(), otherColour.get_alpha());
    }
}
