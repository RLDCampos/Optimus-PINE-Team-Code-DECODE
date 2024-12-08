package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name = "RedAllianceBPlaceAndPark", group = "Pinpoint")
public class RedAllianceBPlaceAndPark extends LinearOpMode {

    // Motors and servos
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, ySliderMotor;
    private Servo clawServo;

    // Odometry and navigation
    private GoBildaPinpointDriver pinpoint;
    private DriveToPoint nav = new DriveToPoint(this);

    // Define positions
    static final Pose2D START_TO_HIGH_CHAMBER = new Pose2D(DistanceUnit.MM, 1000, 0, AngleUnit.DEGREES, 0);
    static final Pose2D HIGH_CHAMBER_TO_PARK = new Pose2D(DistanceUnit.MM, 0, 2000, AngleUnit.DEGREES, 0);

    @Override
    public void runOpMode() {
        // Initialize motors and servos
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        ySliderMotor = hardwareMap.get(DcMotor.class, "y_slider_motor");
        clawServo = hardwareMap.get(Servo.class, "Claw");

        // Motor configurations
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Odometry configuration
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-142.0, 120.0); // Adjust based on hardware
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Update odometry
            pinpoint.update();

            // Task 1: Close claw to hold specimen
            clawServo.setPosition(0.2); // Adjust for closed position
            sleep(500);

            // Task 2: Move slider to high chamber position
            ySliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ySliderMotor.setTargetPosition(1000); // Adjust for actual position
            ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ySliderMotor.setPower(0.5);
            while (ySliderMotor.isBusy() && opModeIsActive()) {
                telemetry.addData("Slider", "Moving to high chamber...");
                telemetry.update();
            }
            ySliderMotor.setPower(0);

            // Task 3: Drive to the high chamber position
            if (nav.driveTo(pinpoint.getPosition(), START_TO_HIGH_CHAMBER, 0.5, 1)) {
                telemetry.addLine("Reached high chamber!");
            }

            // Task 4: Open claw to release specimen
            clawServo.setPosition(0.8); // Adjust for open position
            sleep(500);

            // Task 5: Return slider to starting position
            ySliderMotor.setTargetPosition(0);
            ySliderMotor.setPower(0.5);
            while (ySliderMotor.isBusy() && opModeIsActive()) {
                telemetry.addData("Slider", "Returning to start...");
                telemetry.update();
            }
            ySliderMotor.setPower(0);

            // Task 6: Drive to observation zone
            if (nav.driveTo(pinpoint.getPosition(), HIGH_CHAMBER_TO_PARK, 0.5, 1)) {
                telemetry.addLine("Parked in observation zone!");
            }

            // Final telemetry
            telemetry.addData("Status", "Task Complete");
            telemetry.update();
        }
    }
}
