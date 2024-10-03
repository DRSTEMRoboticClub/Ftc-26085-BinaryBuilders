package org.firstinspires.ftc.teamcode.tools;

import org.firstinspires.ftc.teamcode.tools.Colour;
import java.util.ArrayList;

public class ColourMatcher {
    private ArrayList<Colour> m_colours;

    public void AddColour(Colour newColour) {
        m_colours.add(newColour);
    }

    public void AddColour(String name, double red, double green, double blue, double alpha)
    {
        m_colours.add(new Colour(name, red, green, blue, alpha));
    }

    public Colour ClosestColour(double red, double green, double blue, double alpha) {
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
