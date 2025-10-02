package org.firstinspires.ftc.teamcode;

//package org.firstinspires.ftc.teamcode.util;//

import com.qualcomm.robotcore.hardware.Gamepad;

/** Edge-detect wrapper for FTC Gamepad. Call its methods each loop. */
public class StickyGamepad {
    private final Gamepad gp;

    private boolean lastRightBumper = false;

    public StickyGamepad(Gamepad gp) {
        this.gp = gp;
    }

    /** true only on the transition from not-pressed -> pressed */
    public boolean rightBumperWasPressed() {
        boolean now = gp.right_bumper;
        boolean wasPressed = now && !lastRightBumper;
        lastRightBumper = now;
        return wasPressed;
    }

    // If you want more buttons, add similar methods here...
    // public boolean aWasPressed() { ... }
    // public boolean leftBumperWasPressed() { ... }
}
