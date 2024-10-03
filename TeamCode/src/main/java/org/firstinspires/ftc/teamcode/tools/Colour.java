package org.firstinspires.ftc.teamcode.tools;

public class Colour {
    private final String m_name;
    private final double m_red;
    private final double m_green;
    private final double m_blue;
    private final double m_alpha;

    public Colour(String name, double red, double green, double blue, double alpha) {
        m_name = name;
        m_red = red;
        m_green = green;
        m_blue = blue;
        m_alpha = alpha;
    }

    public double get_red() {
        return m_red;
    }

    public double get_green() {
        return m_green;
    }

    public double get_blue() {
        return m_blue;
    }

    public double get_alpha() {
        return m_alpha;
    }

    public String get_name() {
        return m_name;
    }

    public double get_difference(double red, double green, double blue, double alpha) {
        red /= alpha;
        green /= alpha;
        blue /= alpha;
        double my_red = m_red / m_alpha;
        double my_green = m_green / m_alpha;
        double my_blue = m_blue / m_alpha;
        return Math.max((red - my_red)*(red - my_red), (red - my_red - alpha + m_alpha)*(red - my_red - alpha + m_alpha))
                + Math.max((green - my_green)*(green - my_green), (green - my_green - alpha + m_alpha)*(green - my_green - alpha + m_alpha))
                + Math.max((blue - my_blue)*(blue - my_blue), (blue - my_blue - alpha + m_alpha)*(blue - my_blue - alpha + m_alpha));
    }

    public double get_difference(Colour otherColour) {
        return get_difference(otherColour.get_red(), otherColour.get_green(), otherColour.get_blue(), otherColour.get_alpha());
    }
}
