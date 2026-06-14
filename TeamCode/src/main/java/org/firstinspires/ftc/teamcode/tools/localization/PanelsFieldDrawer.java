package org.firstinspires.ftc.teamcode.tools.localization;

import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresets;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;

/**
 * Draws the robot's live position (and optionally its pose history trail) onto
 * the Panels field view using the Pedro Pathing coordinate preset.
 *
 * Call {@link #init()} once, then {@link #update(Pose)} every loop.
 * If you have a Pedro {@link Follower}, call {@link #update(Follower)} instead to
 * also draw the position history trail.
 *
 * The Panels field widget must be open in the browser for anything to appear.
 * Open http://192.168.43.1:8001 (Control Hub) or http://192.168.49.1:8001 (phone RC).
 */
public class PanelsFieldDrawer {

    // Pedro Pathing coordinate preset maps Pedro inches to the correct field orientation.
    private static final FieldManager field = PanelsField.INSTANCE.getField();

    // Robot body appearance.
    private static final double ROBOT_RADIUS_IN = 7.0;   // circle radius (inches)
    private static final double HEADING_LEN_IN  = 10.0;  // length of heading indicator line
    private static final String ROBOT_FILL      = "#2196F3"; // blue
    private static final String HEADING_COLOR   = "#FFFFFF";
    private static final double HEADING_WIDTH   = 3.0;

    // Trail appearance.
    private static final String TRAIL_COLOR  = "#90CAF9"; // light blue
    private static final double TRAIL_RADIUS = 1.5;       // inch radius of each trail dot

    public static void init() {
        field.setOffsets(FieldPresets.INSTANCE.getPEDRO_PATHING());
        field.setBackground(com.bylazar.field.PanelsField.INSTANCE.getImages().getDECODE().getLIGHT());
    }

    /**
     * Draw just the robot's current pose (no trail). Suitable for TeleOp localizers
     * where there is no Follower available.
     */
    public static void update(double x, double y, double headingRad) {
        field.getCanvas().reset();

        // --- Robot body circle ---
        field.moveCursor(x, y);
        field.setFill(ROBOT_FILL);
        field.clearOutline();
        field.circle(ROBOT_RADIUS_IN);

        // --- Heading indicator line (from center toward front) ---
        double tipX = x + HEADING_LEN_IN * Math.cos(headingRad);
        double tipY = y + HEADING_LEN_IN * Math.sin(headingRad);
        field.moveCursor(x, y);
        field.clearFill();
        field.setOutline(HEADING_COLOR, HEADING_WIDTH);
        field.line(tipX, tipY);

        field.update();
    }

    /**
     * Convenience overload for Pedro {@link Pose}.
     */
    public static void update(Pose pose) {
        update(pose.getX(), pose.getY(), pose.getHeading());
    }

    /**
     * Draw robot + pose history trail from a live Pedro {@link Follower}.
     * The trail shows where the robot has been since the OpMode started.
     */
    public static void update(Follower follower) {
        field.getCanvas().reset();

        // --- Pose history trail (drawn first so robot appears on top) ---
        PoseHistory history = follower.getPoseHistory();
        double[] xs = history.getXPositionsArray();
        double[] ys = history.getYPositionsArray();
        if (xs != null && ys != null) {
            field.setFill(TRAIL_COLOR);
            field.clearOutline();
            int n = Math.min(xs.length, ys.length);
            for (int i = 0; i < n; i++) {
                if (!Double.isNaN(xs[i]) && !Double.isNaN(ys[i])) {
                    field.moveCursor(xs[i], ys[i]);
                    field.circle(TRAIL_RADIUS);
                }
            }
        }

        // --- Robot ---
        Pose pose = follower.getPose();
        update(pose.getX(), pose.getY(), pose.getHeading());
        // Note: update() calls field.update() internally, which sends to browser.
    }
}
