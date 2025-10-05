package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

// ✅ Use the FTC SDK AngleUnit (this is the key fix)
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TestBotDecodeTeleopField", group = "StarterBot")
//@Disabled
public class TestBotDecodeTeleopField extends OpMode {

    // Shooter/Feeder constants
    final double FEED_TIME_SECONDS = 0.30;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    // Drive + mechanisms
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    // IMU for field-centric
    private IMU imu;

    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState launchState;

    // Telemetry helpers
    double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        // Map hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Directions (same as before)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake mode
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // Launcher closed-loop velocity control
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        // Feeders
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        // IMU init (Hub label LEFT, USB UP)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(params);

        telemetry.addData("Status", "Initialized (Field-centric)");
        telemetry.addData("IMU", "Logo=LEFT, USB=UP");
    }

    @Override public void init_loop() { }
    @Override public void start() { imu.resetYaw(); }

    @Override
    public void loop() {
        // --- Field-centric drive ---
        double y  = -gamepad1.left_stick_y;  // forward/back
        double x  =  gamepad1.left_stick_x;  // strafe
        double rx =  gamepad1.right_stick_x; // rotate

        // ✅ Use SDK AngleUnit here
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate joystick vector by -heading (polar method)
        double r = Math.hypot(x, y);
        double joystickAngle = Math.atan2(y, x);
        double fieldAngle = joystickAngle - botHeading;

        double rotatedX = r * Math.cos(fieldAngle);
        double rotatedY = r * Math.sin(fieldAngle);

        double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1.0);
        leftFrontPower  = (rotatedY + rotatedX + rx) / denominator;
        rightFrontPower = (rotatedY - rotatedX - rx) / denominator;
        leftBackPower   = (rotatedY - rotatedX + rx) / denominator;
        rightBackPower  = (rotatedY + rotatedX - rx) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Quick re-zero of yaw
        if (gamepad1.back) imu.resetYaw();

        // Launcher manual control
        if (gamepad1.y)      launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        else if (gamepad1.b) launcher.setVelocity(STOP_SPEED);

        telemetry.addData("Mode", "Field-centric");
        telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
        telemetry.addData("Drive", "LF %.2f RF %.2f LB %.2f RB %.2f",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("Launcher vel", launcher.getVelocity());
        telemetry.addData("State", launchState);
        telemetry.update();
    }

    @Override public void stop() {}

    // Shooter state machine (unchanged)
    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) launchState = LaunchState.SPIN_UP;
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
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}
