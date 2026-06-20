package org.firstinspires.ftc.teamcode.tools.localization;

import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresets;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

/**
 * Draws the robot's live position onto the Panels field view.
 *
 * All network I/O happens on a single daemon background thread — the main
 * OpMode loop is NEVER blocked, even if the browser is slow or not connected.
 * {@link #update} just writes three volatile doubles and returns immediately.
 *
 * Open http://192.168.43.1:8001 (Control Hub) or http://192.168.49.1:8001 (phone RC).
 */
public class PanelsFieldDrawer {

    private static final FieldManager field = PanelsField.INSTANCE.getField();

    private static final double ROBOT_RADIUS_IN = 7.0;
    private static final double HEADING_LEN_IN  = 10.0;
    private static final String ROBOT_FILL      = "#2196F3";
    private static final String HEADING_COLOR   = "#FFFFFF";
    private static final double HEADING_WIDTH   = 3.0;

    // Shared state: main loop writes, background thread reads and draws.
    // Volatile guarantees the background thread sees the latest values without locking.
    private static volatile double pendingX       = 0;
    private static volatile double pendingY       = 0;
    private static volatile double pendingHeading = 0;
    private static volatile boolean pendingUpdate = false;

    // Background daemon thread that owns all Panels API calls.
    // Sleeping 50ms between checks keeps it at ≤20 Hz regardless of call frequency.
    private static final Thread senderThread = new Thread(() -> {
        while (!Thread.currentThread().isInterrupted()) {
            try { Thread.sleep(50); } catch (InterruptedException e) { break; }
            if (!pendingUpdate) continue;
            pendingUpdate = false;
            try {
                double x = pendingX, y = pendingY, h = pendingHeading;
                field.getCanvas().reset();

                // Robot body
                field.moveCursor(x, y);
                field.setFill(ROBOT_FILL);
                field.clearOutline();
                field.circle(ROBOT_RADIUS_IN);

                // Heading line
                double tipX = x + HEADING_LEN_IN * Math.cos(h);
                double tipY = y + HEADING_LEN_IN * Math.sin(h);
                field.moveCursor(x, y);
                field.clearFill();
                field.setOutline(HEADING_COLOR, HEADING_WIDTH);
                field.line(tipX, tipY);

                field.update();   // may block if browser is slow — safely isolated here
            } catch (Throwable t) {
                // Browser not open, socket timeout, network error — ignore and retry next tick.
            }
        }
    }, "PanelsFieldDrawer");

    static {
        senderThread.setDaemon(true);   // JVM/FTC kills it automatically when OpMode ends
        senderThread.start();
    }

    public static void init() {
        try {
            field.setOffsets(FieldPresets.INSTANCE.getPEDRO_PATHING());
            field.setBackground(PanelsField.INSTANCE.getImages().getDECODE().getLIGHT());
        } catch (Throwable t) {
            // Panels library unavailable — drawing will be silently skipped
        }
    }

    /**
     * Non-blocking pose update. Safe to call every loop at any frequency.
     * The background thread draws at ≤20 Hz regardless.
     */
    public static void update(double x, double y, double headingRad) {
        pendingX       = x;
        pendingY       = y;
        pendingHeading = headingRad;
        pendingUpdate  = true;
    }

    public static void update(Pose pose) {
        update(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static void update(Follower follower) {
        Pose pose = follower.getPose();
        update(pose.getX(), pose.getY(), pose.getHeading());
    }
}
