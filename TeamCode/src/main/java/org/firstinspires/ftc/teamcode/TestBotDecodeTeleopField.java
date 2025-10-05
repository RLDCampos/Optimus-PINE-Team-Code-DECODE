//This code was designed for the DECODE season and assumes mecanum field-centric drive, tow feeders and one motor launcher
// imu orientation can be changed in SimplifiedOdometryRobot.java in this project (TeamCode).
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "TestBotDecodeTeleopField", group = "StarterBot")
public class TestBotDecodeTeleopField extends LinearOpMode {

    // --- Shooter/feeder constants (same as your current TeleOp) ---
    private static final double FEED_TIME_SECONDS = 0.30;
    private static final double STOP_SPEED = 0.0;
    private static final double FULL_SPEED = 1.0;
    private static final double LAUNCHER_TARGET_VELOCITY = 1125;
    private static final double LAUNCHER_MIN_VELOCITY = 1075;

    // --- Drive + mechanisms ---
    private SimplifiedOdometryRobot robot;  // owns IMU + drive motors
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    // Launcher state machine (same enum as before)
    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState launchState;
    private double feederStartTime;

    @Override
    public void runOpMode() {
        // Build the robot (this sets motor dirs, initializes IMU ONCE, etc.)
        robot = new SimplifiedOdometryRobot(this);
        robot.initialize(false); // telemetry from TeleOp, not from the robot class

        // Map mechanisms not handled by SimplifiedOdometryRobot
        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Launcher setup (RUN_USING_ENCODER + velocity PIDF)
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        // Feeder baseline
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launchState = LaunchState.IDLE;

        telemetry.addLine("Initialized (Field-centric with SimplifiedOdometryRobot)");
        telemetry.addLine("Press ▶ when ready. BACK = reset heading");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Zero heading at start so current facing = “field forward”
        robot.resetHeading();

        while (opModeIsActive()) {
            // --- Read sensors (updates robot.heading in DEGREES) ---
            robot.readSensors();
            // Convert to radians for joystick rotation
            double headingRad = Math.toRadians(robot.getHeading()); // +CCW

            // --- Driver inputs ---
            double y  = -gamepad1.left_stick_y;  // forward/back
            double x  =  gamepad1.left_stick_x;  // strafe
            double rx =  gamepad1.right_stick_x; // rotate in place

            // Optional slow mode
            double speedScale = gamepad1.right_trigger > 0.5 ? 0.4 : 1.0;
            y  *= speedScale;
            x  *= speedScale;
            rx *= speedScale;

            // --- Field-centric (polar): rotate joystick vector by -heading ---
            double r = Math.hypot(x, y);
            double stickAngle = Math.atan2(y, x);
            double fieldAngle = stickAngle - headingRad;
            double rotatedX = r * Math.cos(fieldAngle); // strafe axis
            double rotatedY = r * Math.sin(fieldAngle); // drive axis

            // If strafing is flipped on your bot, swap the sign:
            // rotatedX = -rotatedX;

            // --- Drive the robot using its low-level mixer ---
            robot.moveRobot(rotatedY, rotatedX, rx);

            // --- Quick re-zero of yaw ---
            if (gamepad1.back) {
                robot.resetHeading();
            }

            // --- Launcher manual velocity toggle (same behavior you had) ---
            if (gamepad1.y) {
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            } else if (gamepad1.b) {
                launcher.setVelocity(STOP_SPEED);
            }

            // --- (Optional) state-machine shot on RB ---
            if (gamepad1.right_bumper) {
                launch(true);
            } else {
                launch(false);
            }

            // Telemetry
            telemetry.addData("Mode", "Field-centric");
            telemetry.addData("Heading (deg)", "%.1f", robot.getHeading());
            telemetry.addData("Drive inputs", "Y %.2f  X %.2f  R %.2f", rotatedY, rotatedX, rx);
            telemetry.addData("Launcher vel", "%.0f", launcher.getVelocity());
            telemetry.addData("Launch state", launchState);
            telemetry.update();
        }

        robot.stopRobot();
        launcher.setVelocity(STOP_SPEED);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

    // ---- Shooter state machine (same logic you had) ----
    private void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederStartTime = getRuntime();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if ((getRuntime() - feederStartTime) > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}
