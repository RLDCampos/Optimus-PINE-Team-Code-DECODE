package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Red Alliance Side A", group = "Autonomous")
public class RedAllianceSideA extends LinearOpMode {

    // Declare hardware
    private DcMotor leftBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor rightFrontMotor;
    private GoBildaPinpointDriver pinpoint;

    // Movement constants
    private static final double MAX_SPEED = 0.6; // Maximum speed multiplier
    private static final double MIN_SPEED = 0.2; // Minimum speed multiplier
    private static final double POSITION_TOLERANCE = 10.0; // Position tolerance in mm
    private static final double HEADING_TOLERANCE = Math.toRadians(1.0); // Heading tolerance in radians

    // Odometry offsets
    private static final double X_POD_OFFSET = -84.0;  // X pod offset in mm
    private static final double Y_POD_OFFSET = -368.0; // Y (strafer) pod offset in mm

    // Starting field position (Red Alliance, near Human Player)
    private static final double START_X = 600.0; // Starting X position in mm
    private static final double START_Y = -1200.0; // Starting Y position in mm
    private static final double START_HEADING = 90.0; // Facing toward center of the field (degrees)

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeMotors();
        initializePinpoint();

        // Set starting position on the field
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, START_X, START_Y, AngleUnit.DEGREES, START_HEADING));

        telemetry.addData("Status", "Waiting for Start...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Movement sequence using precise odometry
            moveToPosition(-500, -1000, 90);  // Example: Move diagonally
            moveToPosition(-300, -500, 180); // Rotate 180° and move backward
            moveToPosition(0, 0, 0);         // Move to center of the field
        }
    }

    private void initializeMotors() {
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");

        // Set motor directions
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor braking behavior
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializePinpoint() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Reset position and IMU
        pinpoint.resetPosAndIMU();

        // Set encoder resolution for goBILDA Swingarm Pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        // Configure pod offsets
        pinpoint.setOffsets(X_POD_OFFSET, Y_POD_OFFSET);
    }

    private void moveToPosition(double targetX, double targetY, double targetHeadingDegrees) {
        boolean atTarget = false;

        while (opModeIsActive() && !atTarget) {
            pinpoint.update(); // Update odometry data
            Pose2D currentPose = pinpoint.getPosition();

            double currentX = currentPose.getX(DistanceUnit.MM);
            double currentY = currentPose.getY(DistanceUnit.MM);
            double currentHeading = currentPose.getHeading(AngleUnit.RADIANS);

            // Calculate errors
            double xError = targetX - currentX;
            double yError = targetY - currentY;
            double headingError = Math.toRadians(targetHeadingDegrees) - currentHeading;

            // Calculate distance to target
            double distanceToTarget = Math.hypot(xError, yError);

            // Adaptive speed based on distance and heading error
            double speedFactor = Math.max(MIN_SPEED, Math.min(MAX_SPEED, distanceToTarget / 500));

            // Check if within tolerance
            if (Math.abs(xError) < POSITION_TOLERANCE &&
                    Math.abs(yError) < POSITION_TOLERANCE &&
                    Math.abs(headingError) < HEADING_TOLERANCE) {
                atTarget = true;
                stopMotors(); // Stop when the target is reached
                break;
            }

            // Compute motor powers
            double forwardPower = speedFactor * yError / Math.hypot(xError, yError);
            double strafePower = speedFactor * xError / Math.hypot(xError, yError);
            double turnPower = speedFactor * headingError;

            // Apply Mecanum drive formulas
            double leftFrontPower = forwardPower + strafePower + turnPower;
            double rightFrontPower = forwardPower - strafePower - turnPower;
            double leftBackPower = forwardPower - strafePower + turnPower;
            double rightBackPower = forwardPower + strafePower - turnPower;

            // Normalize power values
            double maxPower = Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
            );

            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightBackPower /= maxPower;
            }

            // Set motor powers
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);

            // Telemetry for debugging
            telemetry.addData("Target", "X: %.2f Y: %.2f Heading: %.2f",
                    targetX, targetY, targetHeadingDegrees);
            telemetry.addData("Current", "X: %.2f Y: %.2f Heading: %.2f",
                    currentX, currentY, Math.toDegrees(currentHeading));
            telemetry.addData("Errors", "X: %.2f Y: %.2f Heading: %.2f",
                    xError, yError, Math.toDegrees(headingError));
            telemetry.addData("Speed Factor", "%.2f", speedFactor);
            telemetry.update();
        }
    }

    private void stopMotors() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}
