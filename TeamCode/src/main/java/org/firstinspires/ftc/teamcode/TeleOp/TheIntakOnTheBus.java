package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.tools.Colour;
import org.firstinspires.ftc.teamcode.tools.ColourMatcher;

public class TheIntakOnTheBus {
    private final Servo flapWheelServo;

    private final ColorRangeSensor colourSensor;

    private final Telemetry telemetry;

    private ColourMatcher colourMatcher;

    private final String teamColour;

    private enum State
    {
        IDLE,
        GRABBING,
        SPITTING,
        GRABBED
    };

    private State state;

    public TheIntakOnTheBus(Telemetry the_telemetry, Servo flap_servo, ColorRangeSensor sensor, String team_colour)
    {
        flapWheelServo = flap_servo;
        colourSensor = sensor;
        telemetry = the_telemetry;
        teamColour = team_colour;
    }

    public void initialise()
    {
        flapWheelServo.setPosition(0.5);
        state = State.IDLE;
        // Define the colours
        colourMatcher = new ColourMatcher();
        colourMatcher.AddColour("Blue", 0.0, 0.0, 1.0, 1.0);
        colourMatcher.AddColour("Red", 1.0, 0.0, 0.0, 1.0);
        colourMatcher.AddColour("Yellow", 1.0, 1.0, 0.0, 1.0);
    }

    public void grab()
    {
        state = State.GRABBING;
        flapWheelServo.setPosition(1.0);
    }

    public void spit()
    {
        state = State.SPITTING;
        flapWheelServo.setPosition(0.0);
    }

    public void check() throws InterruptedException {
        switch (state)
        {
            case GRABBING:
                if (colourSensor.getDistance(DistanceUnit.CM) < 3)
                {
                    flapWheelServo.setPosition(0.5);
                    String closed_colour = colourMatcher.ClosestColour(new Colour(colourSensor.getNormalizedColors())).get_name();
                    if (closed_colour.equals(teamColour) || closed_colour.equals("Yellow"))
                    {
                        state = State.GRABBED;
                    }
                    else
                    {
                        spit();
                    }
                }
                break;
            case SPITTING:
                if (colourSensor.getDistance(DistanceUnit.CM) > 3)
                {
                    Thread.sleep(200);
                    flapWheelServo.setPosition(0.5);
                    state = State.IDLE;
                }
                break;
        }
    }

}
