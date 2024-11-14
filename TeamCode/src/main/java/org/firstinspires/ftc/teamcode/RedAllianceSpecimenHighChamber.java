package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;
import java.util.Locale;

@Autonomous(name="RedAllianceSpecimenHighChamber", group="Red Alliance")
public class RedAllianceSpecimenHighChamber extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;
    private DcMotor viperSliderMotor;
    private Servo pickupServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // Declare the odometry computer
    private GoBildaPinpointDriver odo;

    private ElapsedTime runtime = new ElapsedTime();

    // AprilTag IDs and locations
    private static final int TAG_ID_REAR_WALL_AUDIENCE = 14;
    private static final int TAG_ID_REAR_WALL_PLAYERS = 15;
    private static final int TAG_ID_REAR_WALL_BACK = 16;

    private double xPos = 0.0;
    private double yPos = 0.0;
    private double heading = 0.0;

    // Define a speed multiplier to reduce the speed
    private static final double SPEED_MULTIPLIER = 0.03;

    // Flag to check if the webcam is available
    private boolean webcamAvailable = false;

    @Override
    public void runOpMode() {

        // Hardware mapping
        leftFrontMotor = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "front_right");
        leftRearMotor = hardwareMap.get(DcMotor.class, "rear_left");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rear_right");
        viperSliderMotor = hardwareMap.get(DcMotor.class, "y_slider_motor");
        pickupServo = hardwareMap.get(Servo.class, "Claw");

        // Initialize the odometry computer (Pinpoint Driver)
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // Motor directions
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize AprilTag processor and check webcam status
        initAprilTagProcessor();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Webcam Available", webcamAvailable ? "Yes" : "No");
        telemetry.update();

        waitForStart();
        runtime.reset();

        moveToPosition(24, 0);
        pickupServo.setPosition(1.0);
        sleep(500);

        if (webcamAvailable) {
            moveToPositionWithAprilTag(60, 0);
        } else {
            telemetry.addLine("Webcam not available, using odometry only");
            telemetry.update();
            moveToPosition(60, 0);  // Use odometry fallback if webcam fails
        }

        turnToHeading(90);
        moveToPosition(60, 12);

        pickupServo.setPosition(0.0);
        sleep(500);

        viperSliderMotor.setPower(0.8);
        sleep(3000);
        viperSliderMotor.setPower(0);

        moveToPosition(0, 0);
        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }

    private void initAprilTagProcessor() {
        try {
            aprilTagProcessor = new AprilTagProcessor.Builder().build();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "webcam_1"))
                    .enableLiveView(true)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(aprilTagProcessor)
                    .build();

            webcamAvailable = true;  // Webcam initialized successfully

        } catch (Exception e) {
            webcamAvailable = false;  // Webcam initialization failed
            telemetry.addData("Error", "Webcam failed to initialize");
            telemetry.update();
        }
    }

    private void moveToPositionWithAprilTag(double targetX, double targetY) {
        moveToPosition(targetX, targetY);

        if (!webcamAvailable) return;  // Skip AprilTag processing if webcam is unavailable

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection tag : detections) {
            if ((tag.id == TAG_ID_REAR_WALL_AUDIENCE || tag.id == TAG_ID_REAR_WALL_PLAYERS || tag.id == TAG_ID_REAR_WALL_BACK) && tag.ftcPose != null) {
                xPos = tag.ftcPose.x;
                yPos = tag.ftcPose.y;
                heading = tag.ftcPose.yaw;

                telemetry.addData("AprilTag Detected:", tag.id);
                telemetry.addData("X Position (inches):", xPos);
                telemetry.addData("Y Position (inches):", yPos);
                telemetry.addData("Heading (degrees):", heading);
                telemetry.update();
                break;
            }
        }
    }

    private void moveToPosition(double targetX, double targetY) {
        xPos = 0.0;
        yPos = 0.0;

        while (opModeIsActive() && (Math.abs(targetX - xPos) > 1 || Math.abs(targetY - yPos) > 1)) {
            odo.update();
            Pose2D position = odo.getPosition();

            xPos = position.getX(DistanceUnit.MM);
            yPos = position.getY(DistanceUnit.MM);
            heading = Math.toDegrees(position.getHeading(AngleUnit.RADIANS));

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
            telemetry.addData("Current X", xPos);
            telemetry.addData("Current Y", yPos);
            telemetry.update();
        }

        stopMotors();
    }

    private void turnToHeading(double targetHeading) {
        while (opModeIsActive() && Math.abs(targetHeading - heading) > 1) {
            odo.update();
            heading = Math.toDegrees(odo.getHeading());

            double headingError = targetHeading - heading;
            double turnPower = SPEED_MULTIPLIER * headingError;

            leftFrontMotor.setPower(-turnPower);
            rightFrontMotor.setPower(turnPower);
            leftRearMotor.setPower(-turnPower);
            rightRearMotor.setPower(turnPower);

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", heading);
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
