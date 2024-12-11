package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "RedAllianceBPlaceAndPark", group = "Autonomous")
public class RedAllianceBPlaceAndPark extends LinearOpMode {
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, ySliderMotor;
    private Servo clawServo;

    private GoBildaPinpointDriver pinpoint;
    private DriveToPoint nav;

    static final Pose2D STARTING_POSITION = new Pose2D(DistanceUnit.MM, 500, 0, AngleUnit.DEGREES, 180);
    static final Pose2D DRIVE_TO_SUBVERSIVE = new Pose2D(DistanceUnit.MM, 500, -1000, AngleUnit.DEGREES, 180);
    static final Pose2D ALIGN_WITH_UPPER_CHAMBER = new Pose2D(DistanceUnit.MM, 500, -1500, AngleUnit.DEGREES, 180);
    static final Pose2D DRIVE_TO_OBSERVATION_ZONE = new Pose2D(DistanceUnit.MM, 1000, -1500, AngleUnit.DEGREES, 180);

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            executeTasks();
        }
    }

    private void initializeHardware() {
        // Initialize motors and servos
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        ySliderMotor = hardwareMap.get(DcMotor.class, "y_slider_motor");
        clawServo = hardwareMap.get(Servo.class, "Claw");

        // Motor configuration
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ySliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Odometry initialization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-90.0, -300.0); // Adjusted offsets
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // Debugging telemetry for odometry
        telemetry.addData("Status", "Pinpoint Initialized");
        telemetry.addData("X offset", pinpoint.getXOffset());
        telemetry.addData("Y offset", pinpoint.getYOffset());
        telemetry.addData("Device Version Number", pinpoint.getDeviceVersion());
        telemetry.addData("Device Scalar", pinpoint.getYawScalar());
        telemetry.update();

        // Navigation initialization
        nav = new DriveToPoint(this);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);
    }

    private void executeTasks() {
        // Task 1: Close claw
        clawServo.setPosition(0.3);
        sleep(500);

        // Task 2: Drive to subversive
        if (driveToPoint(DRIVE_TO_SUBVERSIVE, "Reached subversive!")) {
            // Task 3: Move slider
            moveSlider(500, 0.5, 3000);

            // Task 4: Align with upper chamber
            if (driveToPoint(ALIGN_WITH_UPPER_CHAMBER, "Aligned with upper chamber!")) {
                // Task 5: Lower slider
                moveSlider(0, 0.3, 3000);

                // Task 6: Open claw
                clawServo.setPosition(0.8);
                sleep(500);

                // Task 7: Drive to observation zone
                driveToPoint(DRIVE_TO_OBSERVATION_ZONE, "Parked in observation zone!");
            }
        }

        telemetry.addData("Status", "All tasks completed");
        telemetry.update();
    }

    private boolean driveToPoint(Pose2D targetPose, String successMessage) {
        if (nav.driveTo(pinpoint.getPosition(), targetPose, 0.5, 1.0)) {
            telemetry.addLine(successMessage);
            telemetry.update();
            return true;
        }
        telemetry.addLine("Failed to reach " + successMessage);
        telemetry.update();
        return false;
    }

    private void moveSlider(int targetPosition, double power, long timeoutMs) {
        ySliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ySliderMotor.setTargetPosition(targetPosition);
        ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ySliderMotor.setPower(power);

        long startTime = System.currentTimeMillis();
        while (ySliderMotor.isBusy() && opModeIsActive() && (System.currentTimeMillis() - startTime < timeoutMs)) {
            telemetry.addData("Slider", "Moving to position: %d", targetPosition);
            telemetry.update();
        }
        ySliderMotor.setPower(0);
    }
}







