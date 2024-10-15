package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.tools.Colour;
import java.util.ArrayList;

public class ColourMatcher {
    private ArrayList<Colour> m_colours;

    public ColourMatcher()
    {
        m_colours = new ArrayList<>();
    }

    public void AddColour(Colour newColour) {
        m_colours.add(newColour);
    }

    public void AddColour(String name, int red, int green, int blue, int alpha)
    {
        m_colours.add(new Colour(name, red, green, blue, alpha));
    }

    public Colour ClosestColour(ColorSensor sensor)
    {
        return ClosestColour(sensor.red(), sensor.green(), sensor.blue(), sensor.alpha());
    }

    public Colour ClosestColour(int red, int green, int blue, int alpha) {
        return ClosestColour(new Colour("Unknown", red, green, blue, alpha));
    }

    public Colour ClosestColour(Colour newColour) {
        double diff = Double.POSITIVE_INFINITY;
        Colour closestColour = m_colours.get(0);
        for (Colour c : m_colours)
        {
            double this_diff = c.get_difference(newColour);
            if (this_diff < diff)
            {
                closestColour = c;
                diff = this_diff;
            }
        }
        return closestColour;
    }
}
