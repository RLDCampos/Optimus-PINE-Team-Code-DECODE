package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Odo Precise Movement", group = "Autonomous")
public class OdometryPreciseMovement extends LinearOpMode {

    // Declare hardware and odometry driver
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;
    private GoBildaPinpointDriver odo;

    private static final double SPEED_MULTIPLIER = 0.2; // Motor speed
    //private static final double TICKS_PER_MM = 13.26291192; // Default for Swingarm pod

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeMotors();
        initializeOdometry();

        telemetry.addData("Status", "Waiting for Start...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Movement sequence
            moveAndTurn(0, 600, 0, true);       // Move forward 600 mm, recalibrate IMU
            moveAndTurn(-90, 0, 600, true);    // Rotate CCW 90° and strafe right, recalibrate
            moveAndTurn(180, -300, 0, true);   // Rotate 180° and move backward, recalibrate
            moveAndTurn(90, 300, -300, true);  // Rotate CW 90° and move diagonally, recalibrate
        }
    }

    /**
     * Initializes the motors with appropriate directions.
     */
    private void initializeMotors() {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "front_right");
        leftRearMotor = hardwareMap.get(DcMotor.class, "rear_left");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rear_right");

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Configures the GoBILDA Odometry Computer for use.
     */
    private void initializeOdometry() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Configure odometry with offsets and resolution
        odo.setOffsets(-84.0, -168.0); // Example offsets in mm
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset and calibrate IMU
        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        odo.resetPosAndIMU();
        sleep(1000); // Allow time for calibration
        telemetry.addData("Status", "Odometry Initialized");
        telemetry.update();
    }

    /**
     * Rotates the robot to the specified heading and moves to the specified position.
     * @param targetHeading Target heading in degrees.
     * @param targetX Target X position in mm.
     * @param targetY Target Y position in mm.
     * @param recalibrate Whether to recalibrate the IMU after each action.
     */
    private void moveAndTurn(double targetHeading, double targetX, double targetY, boolean recalibrate) {
        // Rotate to the specified heading
        turnToHeading(targetHeading);

        if (recalibrate) {
            odo.recalibrateIMU();
            telemetry.addData("Status", "IMU Recalibrated After Turn");
            telemetry.update();
        }

        // Move to the specified position
        moveToPosition(targetX, targetY);

        if (recalibrate) {
            odo.recalibrateIMU();
            telemetry.addData("Status", "IMU Recalibrated After Move");
            telemetry.update();
        }
    }

    /**
     * Rotates the robot to the specified heading.
     * @param targetHeadingDegrees Target heading in degrees.
     */
    private void turnToHeading(double targetHeadingDegrees) {
        odo.update();
        double heading = Math.toDegrees(odo.getHeading());

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

    /**
     * Moves the robot to the specified X and Y coordinates.
     * @param targetX Target X position in mm.
     * @param targetY Target Y position in mm.
     */
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

            double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            if (maxPower > 1) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftRearPower /= maxPower;
                rightRearPower /= maxPower;
            }

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

    /**
     * Stops all motors.
     */
    private void stopMotors() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
    }
}
