package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Odometry Auto", group = "Autonomous")
public class OdometryAuto extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;
    private GoBildaPinpointDriver odo;

    private static final double SPEED_MULTIPLIER = 0.2; // Speed for turning and movement
    private static final double TOLERANCE = 1; // Position and heading tolerance in mm and degrees

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeMotors();
        initializeOdometry();

        telemetry.addData("Status", "Waiting for Start...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Movement sequence for a 30-second autonomous
            moveToPosition(600, 0);      // Move forward 600 mm
            recalibrateIMU();           // Recalibrate for precision
            rotateToHeading(-90);       // Rotate CCW to -90°
            moveToPosition(0, 600);     // Strafe to the right 600 mm
            recalibrateIMU();           // Recalibrate for precision
            moveToPosition(300, 600);   // Diagonal movement
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
     * Configures the GoBILDA Odometry Computer.
     */
    private void initializeOdometry() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(-84.0, -168.0); // Example offsets in mm
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        odo.resetPosAndIMU();
        sleep(1000); // Allow time for calibration
        telemetry.addData("Status", "Odometry Initialized");
        telemetry.update();
    }

    /**
     * Moves the robot to the specified X and Y coordinates in mm.
     * @param targetX Target X position in mm.
     * @param targetY Target Y position in mm.
     */
    private void moveToPosition(double targetX, double targetY) {
        odo.update();
        Pose2D currentPosition = odo.getPosition();

        double xPos = currentPosition.getX(DistanceUnit.MM);
        double yPos = currentPosition.getY(DistanceUnit.MM);

        while (opModeIsActive() && (Math.abs(targetX - xPos) > TOLERANCE || Math.abs(targetY - yPos) > TOLERANCE)) {
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

            normalizeMotorPowers(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);

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
     * Rotates the robot to the specified heading in degrees.
     * @param targetHeading Target heading in degrees.
     */
    private void rotateToHeading(double targetHeading) {
        odo.update();
        double heading = Math.toDegrees(odo.getHeading());

        while (opModeIsActive() && Math.abs(targetHeading - heading) > TOLERANCE) {
            odo.update();
            heading = Math.toDegrees(odo.getHeading());

            double headingError = targetHeading - heading;
            double turnPower = SPEED_MULTIPLIER * 0.1 * headingError;

            leftFrontMotor.setPower(-turnPower);
            rightFrontMotor.setPower(turnPower);
            leftRearMotor.setPower(-turnPower);
            rightRearMotor.setPower(turnPower);

            telemetry.addData("Target Heading (degrees)", targetHeading);
            telemetry.addData("Current Heading (degrees)", heading);
            telemetry.update();
        }

        stopMotors();
    }

    /**
     * Recalibrates the IMU for precision.
     */
    private void recalibrateIMU() {
        odo.recalibrateIMU();
        telemetry.addData("Status", "IMU Recalibrated");
        telemetry.update();
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

    /**
     * Normalizes motor powers to ensure they are within the range [-1, 1].
     */
    private void normalizeMotorPowers(double... powers) {
        double maxPower = 0;
        for (double power : powers) {
            maxPower = Math.max(maxPower, Math.abs(power));
        }
        if (maxPower > 1) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= maxPower;
            }
        }
    }
}
