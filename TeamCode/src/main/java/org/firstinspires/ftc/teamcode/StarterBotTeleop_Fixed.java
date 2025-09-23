package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotTeleop_Fixed", group = "StarterBot")
public class StarterBotTeleop_Fixed extends OpMode {

    // ---- Tunables ----
    final double FEED_TIME_SECONDS = 0.20;     // How long feeders run per shot
    final double STOP_SPEED = 0.0;             // CRServo stop power
    final double FULL_SPEED = 1.0;             // CRServo feed power

    // Velocity control targets (ticks/second for REV encoders)
    final double LAUNCHER_TARGET_VELOCITY = 1125;  // your target setpoint
    final double LAUNCHER_MIN_VELOCITY = 1075;     // must exceed before feeding
    final double SPINUP_SETTLE_TIME = 0.12;        // seconds above min before feed

    // Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // State
    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState launchState;

    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime spinUpTimer = new ElapsedTime();

    // Drive telemetry
    double leftPower;
    double rightPower;

    // Input edge detection
    private boolean prevRightBumper = false;
    private boolean prevRightTriggerActive = false;
    private final double TRIGGER_THRESHOLD = 0.55;

    // Helper: get "was-just-pressed" from bumper or trigger
    private boolean shotRequestedThisLoop() {
        boolean bumperNow = gamepad1.right_bumper;
        boolean triggerNow = gamepad1.right_trigger > TRIGGER_THRESHOLD;

        boolean bumperRising = bumperNow && !prevRightBumper;
        boolean triggerRising = triggerNow && !prevRightTriggerActive;

        prevRightBumper = bumperNow;
        prevRightTriggerActive = triggerNow;

        return bumperRising || triggerRising;
    }

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        leftDrive   = hardwareMap.get(DcMotor.class,   "left_drive");
        rightDrive  = hardwareMap.get(DcMotor.class,   "right_drive");
        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(CRServo.class,   "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class,   "right_feeder");

        // Drive directions (flip if your bot drives backward on first test)
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Launcher velocity mode
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);

        // If your velocity reads negative while spinning the desired direction,
        // you can either flip direction OR just use Math.abs(...) in checks (we do that).
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        // Drive brake makes stopping crisp
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        // Initialize feeders stopped
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        // PIDF for RUN_USING_ENCODER (starter values; feel free to tune)
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        // Make both feeders push the same direction (reverse one side)
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override public void init_loop() {}

    @Override public void start() {}

    @Override
    public void loop() {
        // Arcade drive
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Optional manual flywheel control (Y = spin, B = stop)
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) {
            launcher.setVelocity(0);
        }

        // State machine: request a shot on rising edge of bumper OR trigger
        boolean shotRequested = shotRequestedThisLoop();
        launch(shotRequested);

        // Telemetry for teaching/debugging
        double vel = launcher.getVelocity();
        telemetry.addData("State", launchState);
        telemetry.addData("Drive", "L %.2f  R %.2f", leftPower, rightPower);
        telemetry.addData("Launcher vel (tps)", "%.1f", vel);
        telemetry.addData("Above Min?", Math.abs(vel) > LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Inputs", "RB:%s  RT:%.2f (>%3.2f fires)",
                gamepad1.right_bumper, gamepad1.right_trigger, TRIGGER_THRESHOLD);
        telemetry.addData("Shot request (edge)", shotRequested);
    }

    @Override public void stop() {
        launcher.setVelocity(0);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

    private void arcadeDrive(double forward, double rotate) {
        leftPower  = forward + rotate;
        rightPower = forward - rotate;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    private void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                    spinUpTimer.reset();
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                }
                break;

            case SPIN_UP:
                // Keep commanding the target in case other buttons changed it
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                // Require absolute velocity above threshold continuously for a short time
                if (Math.abs(launcher.getVelocity()) > LAUNCHER_MIN_VELOCITY) {
                    if (spinUpTimer.seconds() >= SPINUP_SETTLE_TIME) {
                        launchState = LaunchState.LAUNCH;
                    }
                } else {
                    // Not yet at speed, reset the settle timer
                    spinUpTimer.reset();
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    // Stop feeders; leave flywheel spinning so back-to-back shots feel snappy.
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
}
