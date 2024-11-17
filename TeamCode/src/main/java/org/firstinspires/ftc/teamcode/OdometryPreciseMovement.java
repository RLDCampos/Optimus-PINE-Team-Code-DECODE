package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "OdometryPreciseMovement", group = "Autonomous")
public class OdometryPreciseMovement extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;
    private GoBildaPinpointDriver odo;

    private static final double SPEED_MULTIPLIER = 0.2; // Motor speed
    private static final double TICKS_PER_MM = 13.26291192; // Default for Swingarm pod

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFrontMotor = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "front_right");
        leftRearMotor = hardwareMap.get(DcMotor.class, "rear_left");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rear_right");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Configure odometry computer
        odo.setOffsets(-84.0, -168.0); // Offsets in mm (example values, adjust as needed)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD); // Using swingarm pods
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset position and calibrate IMU
        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        odo.resetPosAndIMU();
        sleep(1000); // Allow time for calibration

        telemetry.addData("Status", "Calibrated and Initialized");
        telemetry.update();

        // Set motor directions
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        if (opModeIsActive()) {
            // Move to position (600 mm forward, 0 mm lateral)
            moveToPosition(600, 0);

            // Pause for 2 seconds
            sleep(2000);

            // Move back to position (-300 mm forward, 0 mm lateral)
            moveToPosition(-300, 0);

            //Pause for 1 seconds
            sleep(1000);

            // Rotate 90 degrees counterclockwise
            turnToHeading(-90); // 90 degrees clockwise

            //Pause for 1 seconds
            sleep(1000);

            // Move to position (1200 mm forward, 0 mm lateral)
            moveToPosition(1200, 0);

            //Pause for 1 seconds
            sleep(1000);

            // Rotate 90 degrees counterclockwise
            turnToHeading(-90); // 90 degrees clockwise

            // Move to position (600 mm forward, 0 mm lateral)
            moveToPosition(600, 0);

            //Pause for 5 seconds
            sleep(5000);

            // Move back to position (-300 mm forward, 0 mm lateral)
            moveToPosition(-300, 0);

            // Rotate 90 degrees counterclockwise
            turnToHeading(-90); // 90 degrees clockwise

            // Move to position (1200 mm forward, 0 mm lateral)
            moveToPosition(1200, 0);

            // Rotate 90 degrees counterclockwise
            turnToHeading(-90); // 90 degrees clockwise

            // Move to position (120 mm forward, 0 mm lateral)
            moveToPosition(120, 0);

            // Rotate 5 full turns
            //turnToHeading(1800); // 360 degrees * 5 turns

            // Recalibrate IMU during autonomous
            odo.recalibrateIMU();
        }
    }

    private void moveToPosition(double targetX, double targetY) {
        odo.update();
        Pose2D currentPosition = odo.getPosition();

        double xPos = currentPosition.getX(DistanceUnit.MM);
        double yPos = currentPosition.getY(DistanceUnit.MM);

        while (opModeIsActive() && (Math.abs(targetX - xPos) > 1 || Math.abs(targetY - yPos) > 1)) {
            odo.update();
            currentPosition = odo.getPosition();
            xPos = currentPosition.getX(DistanceUnit.MM);
            yPos = currentPosition.getY(DistanceUnit.MM);

            double xError = targetX - xPos;
            double yError = targetY - yPos;

            double leftFrontPower = SPEED_MULTIPLIER * (yError + xError);
            double rightFrontPower = SPEED_MULTIPLIER * (yError - xError);
            double leftRearPower = leftFrontPower;
            double rightRearPower = rightFrontPower;

            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftRearMotor.setPower(leftRearPower);
            rightRearMotor.setPower(rightRearPower);

            telemetry.addData("Target X (mm)", targetX);
            telemetry.addData("Target Y (mm)", targetY);
            telemetry.addData("Current X (mm)", xPos);
            telemetry.addData("Current Y (mm)", yPos);
            telemetry.update();
        }

        stopMotors();
    }

    private void turnToHeading(double targetHeadingDegrees) {
        odo.update();
        double heading = Math.toDegrees(odo.getHeading()); // Convert radians to degrees

        while (opModeIsActive() && Math.abs(targetHeadingDegrees - heading) > 1) {
            odo.update();
            heading = Math.toDegrees(odo.getHeading());

            double headingError = targetHeadingDegrees - heading;
            double turnPower = SPEED_MULTIPLIER * 0.1 * headingError;

            leftFrontMotor.setPower(-turnPower);
            rightFrontMotor.setPower(turnPower);
            leftRearMotor.setPower(-turnPower);
            rightRearMotor.setPower(turnPower);

            telemetry.addData("Target Heading (degrees)", targetHeadingDegrees);
            telemetry.addData("Current Heading (degrees)", heading);
            telemetry.update();
        }

        stopMotors();
    }

    private void stopMotors() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
    }
}
