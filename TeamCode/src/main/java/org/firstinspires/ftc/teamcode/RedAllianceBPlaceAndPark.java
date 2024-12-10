package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "RedAllianceBPlaceAndPark", group = "Pinpoint")
public class RedAllianceBPlaceAndPark extends LinearOpMode {

    // Declare OpMode members for motors and servos
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor ySliderMotor = null;
    private Servo clawServo = null;

    // Declare OpMode members for odometry and navigation
    private GoBildaPinpointDriver pinpoint; // Odometry Computer
    private DriveToPoint nav = new DriveToPoint(this); // Point-to-point navigation class

    // Define positions
    static final Pose2D STARTING_POSITION = new Pose2D(DistanceUnit.MM, 500, 0, AngleUnit.DEGREES, 180); // Facing the wall
    static final Pose2D DRIVE_TO_SUBVERSIVE = new Pose2D(DistanceUnit.MM, 500, -1000, AngleUnit.DEGREES, 180); // Backward to subversive
    static final Pose2D ALIGN_WITH_UPPER_CHAMBER = new Pose2D(DistanceUnit.MM, 500, -1500, AngleUnit.DEGREES, 180); // Align to place specimen
    static final Pose2D DRIVE_TO_OBSERVATION_ZONE = new Pose2D(DistanceUnit.MM, 1000, -1500, AngleUnit.DEGREES, 180); // Strafe to park

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
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
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

            // Task 2: Drive backward towards the subversive
            if (nav.driveTo(pinpoint.getPosition(), DRIVE_TO_SUBVERSIVE, 0.7, 1)) {
                telemetry.addLine("Reached subversive!");
                telemetry.update();
            }

            // Task 3: Move slider to upper chamber position
            ySliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ySliderMotor.setTargetPosition(500); // Adjust for actual position
            ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ySliderMotor.setPower(0.5);
            long startTime = System.currentTimeMillis();
            while (ySliderMotor.isBusy() && opModeIsActive() && (System.currentTimeMillis() - startTime < 3000)) {
                telemetry.addData("Slider", "Moving to high chamber...");
                telemetry.update();
            }
            ySliderMotor.setPower(0);

            // Task 4: Align with upper chamber (2D robot positioning)
            if (nav.driveTo(pinpoint.getPosition(), ALIGN_WITH_UPPER_CHAMBER, 0.3, 2)) {
                telemetry.addLine("Aligned with upper chamber!");
                telemetry.update();
            }

            // Task 5: Move slider back down slowly
            ySliderMotor.setTargetPosition(500); // Adjust for actual position
            ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ySliderMotor.setPower(0.3);
            while (ySliderMotor.isBusy() && opModeIsActive()) {
                telemetry.addData("Slider", "Lowering slowly...");
                telemetry.update();
            }
            ySliderMotor.setPower(0);

            // Task 6: Open claw to release specimen
            clawServo.setPosition(0.8); // Adjust for open position
            sleep(500);

            // Step 1: Move forward slightly to clear the subversive
            nav.driveTo(pinpoint.getPosition(), new Pose2D(DistanceUnit.MM, 500, -1400, AngleUnit.DEGREES, 180), 0.5, 1);

            // Step 2: Rotate to face 0° heading
            nav.driveTo(pinpoint.getPosition(), new Pose2D(DistanceUnit.MM, 500, -1400, AngleUnit.DEGREES, 0), 0.3, 1);

            // Step 3: Drive to the observation zone
            if (nav.driveTo(pinpoint.getPosition(), DRIVE_TO_OBSERVATION_ZONE, 0.3, 0.8)) {
                telemetry.addLine("Parked in observation zone!");
                telemetry.update();
            }


            // Final telemetry
            telemetry.addData("Status", "Task Complete");
            telemetry.update();
        }
    }
}


