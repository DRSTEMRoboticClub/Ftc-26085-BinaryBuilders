package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TheIntakOnTheArm {
    private final DcMotor flapWheelMotor;

    private final ColorRangeSensor colourSensor;

    private final Telemetry telemetry;

    private ColourMatcher colourMatcher;

    private final String teamColour;

    public enum State
    {
        IDLE,
        GRABBING,
        SPITTING,
        GRABBED
    };

    public State getState()
    {
        return state;
    }

    private State state;

    public TheIntakOnTheArm(Telemetry the_telemetry, DcMotor flap_servo, ColorRangeSensor sensor, String team_colour)
    {
        flapWheelMotor = flap_servo;
        colourSensor = sensor;
        telemetry = the_telemetry;
        teamColour = team_colour;
    }

    public void initialise()
    {
        // State
        state = State.IDLE;

        // Flap wheel
        flapWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flapWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flapWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define the colours
        colourMatcher = new ColourMatcher();
        colourMatcher.AddColour("Blue", 50, 130, 400, 200);
        colourMatcher.AddColour("Red", 340, 200, 120, 219);
        colourMatcher.AddColour("Yellow", 500, 880, 250, 541);
    }

    public void grab()
    {
        state = State.GRABBING;
        flapWheelMotor.setPower(1.0);
    }

    public void spit()
    {
        state = State.SPITTING;
        flapWheelMotor.setPower(-1.0);
    }

    public void stop()
    {
        state = State.IDLE;
        flapWheelMotor.setPower(0.f);
    }

    public void update() throws InterruptedException {
        telemetry.addData("Intake Range", colourSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Intake Colour", colourMatcher.ClosestColour(colourSensor).get_name());
        telemetry.addData("Intake Red:", colourSensor.red());
        telemetry.addData("Intake Green:", colourSensor.green());
        telemetry.addData("Intake Blue:", colourSensor.blue());
        telemetry.addData("Intake Alpha:", colourSensor.alpha());

        switch (state)
        {
            case GRABBING:
                if (colourSensor.getDistance(DistanceUnit.CM) < 3)
                {
                    String closed_colour = colourMatcher.ClosestColour(colourSensor).get_name();
                    if (closed_colour.equals(teamColour) || closed_colour.equals("Yellow"))
                    {
                        telemetry.addData("Intake", "Grabbed");
                        state = State.GRABBED;
                        Thread.sleep(1000);
                        flapWheelMotor.setPower(0.0);
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
                    Thread.sleep(1000);
                    flapWheelMotor.setPower(0.0);
                    state = State.IDLE;
                    grab();
                }
                break;
        }
    }

}
