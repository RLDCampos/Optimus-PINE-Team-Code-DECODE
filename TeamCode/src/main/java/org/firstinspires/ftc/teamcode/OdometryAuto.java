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

    private static final double TOLERANCE = 1.0; // Position and heading tolerance in mm and degrees

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeMotors();
        initializeOdometry();

        telemetry.addData("Status", "Waiting for Start...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Example of an autonomous path
            moveToTarget(600, 0, 0);      // Move forward to (600 mm, 0 mm) with 0° heading
            recalibrateIMU();            // Recalibrate for precision
            moveToTarget(0, -600, -90);   // Strafe right to (0 mm, 600 mm) with -90° heading
            recalibrateIMU();            // Recalibrate for precision
            moveToTarget(300, 600, 180); // Diagonal movement to (300 mm, 600 mm) with 180° heading
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
     * Configures the GoBILDA Pinpoint Odometry Computer.
     */
    private void initializeOdometry() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

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
     * Moves the robot to the specified X, Y, and heading using cosine scaling for smooth motion.
     * @param targetX Target X position in mm.
     * @param targetY Target Y position in mm.
     * @param targetHeading Target heading in degrees.
     */
    private void moveToTarget(double targetX, double targetY, double targetHeading) {
        odo.update();
        Pose2D currentPosition = odo.getPosition();

        double xPos = currentPosition.getX(DistanceUnit.MM);
        double yPos = currentPosition.getY(DistanceUnit.MM);
        double heading = Math.toDegrees(odo.getHeading());

        double maxError = 1000;  // Max positional error (mm)
        double maxPower = 0.8;   // Maximum motor power
        double maxHeadingError = 90; // Max heading error (degrees)
        double headingPowerScale = 0.2; // Scaling factor for heading adjustment

        while (opModeIsActive() && (
                Math.abs(targetX - xPos) > TOLERANCE ||
                        Math.abs(targetY - yPos) > TOLERANCE ||
                        Math.abs(targetHeading - heading) > TOLERANCE)) {

            odo.update();
            currentPosition = odo.getPosition();
            xPos = currentPosition.getX(DistanceUnit.MM);
            yPos = currentPosition.getY(DistanceUnit.MM);
            heading = Math.toDegrees(odo.getHeading());

            // Calculate errors
            double xError = targetX - xPos;
            double yError = targetY - yPos;
            double hError = targetHeading - heading;

            // Calculate motor powers using cosine scaling
            double leftFrontPower = calculateCosinePower(yError + xError, maxError, maxPower)
                    + headingPowerScale * calculateCosinePower(hError, maxHeadingError, maxPower);

            double rightFrontPower = calculateCosinePower(yError - xError, maxError, maxPower)
                    - headingPowerScale * calculateCosinePower(hError, maxHeadingError, maxPower);

            double leftRearPower = calculateCosinePower(yError + xError, maxError, maxPower)
                    + headingPowerScale * calculateCosinePower(hError, maxHeadingError, maxPower);

            double rightRearPower = calculateCosinePower(yError - xError, maxError, maxPower)
                    - headingPowerScale * calculateCosinePower(hError, maxHeadingError, maxPower);

            // Normalize all motor powers
            normalizeMotorPowers(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);

            // Apply motor powers
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftRearMotor.setPower(leftRearPower);
            rightRearMotor.setPower(rightRearPower);


            // Telemetry for debugging
            telemetry.addData("Target X (mm)", targetX);
            telemetry.addData("Target Y (mm)", targetY);
            telemetry.addData("Target Heading (degrees)", targetHeading);
            telemetry.addData("Current X (mm)", xPos);
            telemetry.addData("Current Y (mm)", yPos);
            telemetry.addData("Current Heading (degrees)", heading);
            telemetry.update();
        }

        stopMotors();
    }

    /**
     * Calculates motor power using a cosine scaling function for smooth transitions.
     */
    private double calculateCosinePower(double error, double maxError, double maxPower) {
        if (Math.abs(error) > maxError) {
            return Math.signum(error) * maxPower; // Maximum power for large errors
        }
        // Cosine scaling for smoother transitions
        double scaledPower = maxPower * Math.cos((Math.PI * error) / (2 * maxError));
        return Math.signum(error) * Math.max(Math.abs(scaledPower), 0.1); // Ensure a minimum power
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
