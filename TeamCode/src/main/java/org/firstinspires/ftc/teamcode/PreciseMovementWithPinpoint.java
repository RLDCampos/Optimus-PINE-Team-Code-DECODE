package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

@Autonomous(name = "PreciseMovementWithPinpoint", group = "Test")
public class PreciseMovementWithPinpoint extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;
    private GoBildaPinpointDriver odo; // The Pinpoint odometry computer

    private static final double SPEED_MULTIPLIER = 0.2; // Set for slow speed
    private static final double TICKS_PER_MM = 19.894;
    private static final double INCHES_TO_MM = 25.4;

    @Override
    public void runOpMode() {
        // Hardware initialization
        leftFrontMotor = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "front_right");
        leftRearMotor = hardwareMap.get(DcMotor.class, "rear_left");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rear_right");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Configure the odometry computer with goBILDA specs and two dead wheels
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderResolution(TICKS_PER_MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset the position and recalibrate the IMU
        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        odo.resetPosAndIMU(); // Reset position and calibrate IMU
        sleep(1000); // Give time for IMU calibration

        telemetry.addData("Status", "Calibrated and Initialized");
        telemetry.update();

        // Motor directions
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // Move 24 inches forward
        moveToPosition(24 * INCHES_TO_MM, 0);
        sleep(2000);

        // Move laterally to the right (strafe)
        moveToPosition(24 * INCHES_TO_MM, 24 * INCHES_TO_MM);

        // Rotate around the center five times
        turnToHeading(360 * 5);

        // Optional: recalibrate IMU during the match without resetting position
        odo.recalibrateIMU();
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

            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Current X", String.format(Locale.US, "%.2f", xPos));
            telemetry.addData("Current Y", String.format(Locale.US, "%.2f", yPos));
            telemetry.update();
        }

        stopMotors();
    }

    private void turnToHeading(double targetHeading) {
        odo.update();
        double heading = Math.toDegrees(odo.getHeading());

        while (opModeIsActive() && Math.abs(targetHeading - heading) > 1) {
            odo.update();
            heading = Math.toDegrees(odo.getHeading());

            double headingError = targetHeading - heading;
            double turnPower = SPEED_MULTIPLIER * 0.1 * headingError;

            leftFrontMotor.setPower(-turnPower);
            rightFrontMotor.setPower(turnPower);
            leftRearMotor.setPower(-turnPower);
            rightRearMotor.setPower(turnPower);

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", String.format(Locale.US, "%.2f", heading));
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
