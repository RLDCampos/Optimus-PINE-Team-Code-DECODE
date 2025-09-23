package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "StarterBotAuto_Simple", group = "StarterBot")
public class StarterBotAuto_Simple extends LinearOpMode {

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double SPINUP_SETTLE_TIME = 0.15;
    final double FEED_TIME_SECONDS = 0.20;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftDrive  = hardwareMap.get(DcMotor.class,   "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class,   "right_drive");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        CRServo leftFeeder  = hardwareMap.get(CRServo.class,  "left_feeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class,  "right_feeder");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        waitForStart();
        if (isStopRequested()) return;

        // Spin up
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        ElapsedTime settle = new ElapsedTime();
        settle.reset();

        while (opModeIsActive()) {
            double v = Math.abs(launcher.getVelocity());
            if (v > LAUNCHER_MIN_VELOCITY) {
                if (settle.seconds() >= SPINUP_SETTLE_TIME) break;
            } else {
                settle.reset();
            }
            telemetry.addData("Launcher vel", v);
            telemetry.update();
        }

        // Feed one piece
        ElapsedTime feedTimer = new ElapsedTime();
        leftFeeder.setPower(1.0);
        rightFeeder.setPower(1.0);
        while (opModeIsActive() && feedTimer.seconds() < FEED_TIME_SECONDS) {
            // hold feed
        }
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        // Optionally drive forward a bit
        leftDrive.setPower(0.3);
        rightDrive.setPower(0.3);
        sleep(600);
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Stop flywheel
        launcher.setVelocity(0);
    }
}

