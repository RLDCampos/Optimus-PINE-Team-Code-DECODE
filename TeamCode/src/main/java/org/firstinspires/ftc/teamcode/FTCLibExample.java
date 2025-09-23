package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="FTCLibExample", group="Examples")
public class FTCLibExample extends OpMode {

    private GamepadEx driverOp;

    @Override
    public void init() {
        // Wrap gamepad1 with FTCLib's helper
        driverOp = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        // Example: detect a single-press on the right bumper
        if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            telemetry.addData("Button", "Right bumper just pressed!");
        }

        // Example: continuous trigger read
        double rt = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        telemetry.addData("Right Trigger", rt);

        driverOp.readButtons(); // IMPORTANT: call at end of loop to update button states
    }
}

