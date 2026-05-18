package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoSelector {
    public enum Alliance { BLUE, RED }
    public enum StartPos { LEFT, RIGHT }
    public enum Strategy { SCORE_PARK, CYCLE }

    private Alliance alliance = Alliance.BLUE;
    private StartPos startPos = StartPos.LEFT;
    private Strategy strategy = Strategy.SCORE_PARK;
    private int delay = 0;

    private boolean lastUp, lastDown, lastLeft, lastRight;

    public void update(Gamepad gp) {
        if (gp.dpad_up && !lastUp) delay++;
        if (gp.dpad_down && !lastDown && delay > 0) delay--;
        
        if (gp.x) alliance = Alliance.BLUE;
        if (gp.b) alliance = Alliance.RED;
        
        if (gp.dpad_left) startPos = StartPos.LEFT;
        if (gp.dpad_right) startPos = StartPos.RIGHT;
        
        if (gp.y) strategy = Strategy.CYCLE;
        if (gp.a) strategy = Strategy.SCORE_PARK;

        lastUp = gp.dpad_up;
        lastDown = gp.dpad_down;
    }

    public void display(Telemetry telemetry) {
        telemetry.addLine("=== AUTO SELECTOR ===");
        telemetry.addData("Alliance (X/B)", alliance);
        telemetry.addData("Start Pos (L/R)", startPos);
        telemetry.addData("Strategy (Y/A)", strategy);
        telemetry.addData("Delay (Up/Dn)", delay);
        telemetry.addLine("=====================");
    }

    public Alliance getAlliance() { return alliance; }
    public StartPos getStartPos() { return startPos; }
    public Strategy getStrategy() { return strategy; }
    public int getDelay() { return delay; }
}
