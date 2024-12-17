package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

@Autonomous(name = "Red AllianceB Place and Park", group = "Autonomous")
public class RedAllianceBPlaceAndPark extends LinearOpMode {

    // Motors and servo
    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, ySliderMotor;
    Servo clawServo;

    // Odometry and navigation
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    // Target positions for navigation
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM, -560, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, -670, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, -100, -650, AngleUnit.DEGREES, 90);

    // State machine for managing autonomous sequence
    enum StateMachine {
        WAITING_FOR_START,
        CLOSE_CLAW,
        DRIVE_TO_TARGET_1,
        SLIDER_UP,
        DRIVE_TO_TARGET_2,
        SLIDER_DOWN_AND_OPEN_CLAW,
        DRIVE_TO_TARGET_3,
        AT_TARGET
    }

    @Override
    public void runOpMode() {
        // Hardware initialization
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        ySliderMotor = hardwareMap.get(DcMotor.class, "y_slider_motor");
        clawServo = hardwareMap.get(Servo.class, "Claw");

        // Configure motor directions and braking
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ySliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Odometry initialization
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-75.54, -60.43);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,  // x odometry pod direction
                GoBildaPinpointDriver.EncoderDirection.REVERSED  // y odometry pod direction
        );

        odo.resetPosAndIMU();

        // Navigation initialization
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);


        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update(); // Update odometry data

            switch (stateMachine) {
                case WAITING_FOR_START:
                    stateMachine = StateMachine.CLOSE_CLAW;
                    break;

                case CLOSE_CLAW:
                    clawServo.setPosition(0.2); // Close claw
                    sleep(500);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;

                case DRIVE_TO_TARGET_1:
                    if (nav.driveTo(odo.getPosition(), TARGET_1, 0.7, 0)) {
                        telemetry.addLine("Reached Position #1!");
                        stateMachine = StateMachine.SLIDER_UP;
                    }
                    break;

                case SLIDER_UP:
                    moveSlider(130); // Move slider up
                    stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    break;

                case DRIVE_TO_TARGET_2:
                    if (nav.driveTo(odo.getPosition(), TARGET_2, 0.7, 0)) {
                        telemetry.addLine("Reached Position #2!");
                        stateMachine = StateMachine.SLIDER_DOWN_AND_OPEN_CLAW;
                    }
                    break;

                case SLIDER_DOWN_AND_OPEN_CLAW:
                    moveSliderAndOpenClaw(30);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    break;

                case DRIVE_TO_TARGET_3:
                    if (nav.driveTo(odo.getPosition(), TARGET_3, 0.7, 0)) {
                        telemetry.addLine("Reached Position #3!");
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;

                case AT_TARGET:
                    telemetry.addLine("Autonomous Complete!");
                    telemetry.update();
                    break;
            }

            // Apply calculated motor power
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            // Update telemetry
            telemetry.addData("State", stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();
        }
    }

    private void moveSlider(int mm) {
        int ticks = mm * 10;
        ySliderMotor.setTargetPosition(ySliderMotor.getCurrentPosition() + ticks);
        ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ySliderMotor.setPower(0.5);

        while (ySliderMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Y Slider Position", ySliderMotor.getCurrentPosition());
            telemetry.update();
        }
        ySliderMotor.setPower(0);
    }

    private void moveSliderAndOpenClaw(int mm) {
        int ticks = Math.abs(mm) * 10;
        int targetPosition = ySliderMotor.getCurrentPosition() - ticks;
        ySliderMotor.setTargetPosition(targetPosition);
        ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ySliderMotor.setPower(-0.4);

        boolean clawOpened = false;

        while (ySliderMotor.isBusy() && opModeIsActive()) {
            if (!clawOpened) {
                clawServo.setPosition(0.2); // Gradual claw opening
                clawOpened = true;
            }
        }
        ySliderMotor.setPower(0);
        clawServo.setPosition(0); // Fully open claw
    }
}
