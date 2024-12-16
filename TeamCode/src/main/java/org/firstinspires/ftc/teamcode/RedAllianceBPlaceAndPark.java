package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Red AllianceB Place and Park Smooth", group = "Autonomous")
public class RedAllianceBPlaceAndPark extends LinearOpMode {

    // Motors and servo
    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, ySliderMotor;
    Servo clawServo;

    // Odometry and navigation
    GoBildaPinpointDriver odo;
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    // Target poses
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM, -560, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, -670, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, -100, 650, AngleUnit.DEGREES, 90);

    // State machine for managing autonomous sequence
    enum StateMachine {
        WAITING_FOR_START,
        CLOSE_CLAW,
        DRIVE_TO_TARGET_1,
        SLIDER_UP,
        DRIVE_TO_TARGET_2,
        SLIDER_DOWN_AND_OPEN_CLAW,
        DRIVE_TO_TARGET_3,
        AT_TARGET
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware initialization
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        ySliderMotor = hardwareMap.get(DcMotor.class, "y_slider_motor");
        clawServo = hardwareMap.get(Servo.class, "Claw");

        // Configure motor directions and braking
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ySliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Odometry initialization
        //The X Pod Offset measures how far (in mm) sideways the X pod is from the tracking point. Left of the point
        //is a positive number, right of the point is a negative number.
        //The Y Pod Offset measures how far forwards the Y pod is from the tracking point. Forwards of the point
        //is positive, backwards is negative. In this example, to track the center of your robot, the X offset should be
        //-84mm, and the Y offset should be -168mm.
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(75.54, -60.43);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,// x odometry pod direction
                GoBildaPinpointDriver.EncoderDirection.FORWARD);// y odometry pod directio


        odo.resetPosAndIMU();

        // Navigation initialization
        nav = new org.firstinspires.ftc.teamcode.DriveToPoint(this);
        //If p (Proportional Coefficient Controls how strongly the robot
        // reacts to positional errors in the X and Y directions.
        // is to high expect oscillations or the robot will overshoot the target.
        //If p is too low, the robot will take too long to reach the target or stop short.
        // Start p = 0.01 and increase it if the robot is too slow.
        //If d (Derivative Coefficient) is too high, it can cause the robot to hesitate or move jerkily.
        //If d is too low, the robot may overshoot the target or oscillate.
        // Start d = 0.001 and increase it if the robot is too slow.
        //nav.setXYCoefficients(0.01, 0.001, 0.001, DistanceUnit.MM, 15);
        //Acceleration specifies the feedforward term for acceleration control in navigation.
        //Start with 0.001 and increase it if the robot feels sluggish
        // when starting or transitioning between movement
        //Tolerance defines the acceptable margin of error in position for considering the robot "at the target."
        //nav.setYawCoefficients(0.5, 0.0, 0.1, AngleUnit.DEGREES, 3);
        nav.setDriveType(org.firstinspires.ftc.teamcode.DriveToPoint.DriveType.MECANUM);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // State machine logic
        StateMachine stateMachine = StateMachine.WAITING_FOR_START;

        while (opModeIsActive()) {
            odo.update(); // Update odometry data

            switch (stateMachine) {
                case WAITING_FOR_START:
                    odo.resetPosAndIMU(); // Reset odometry and IMU
                    stateMachine = StateMachine.CLOSE_CLAW;
                    break;

                case CLOSE_CLAW:
                    clawServo.setPosition(0.2); // Close claw
                    sleep(500);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;

                case DRIVE_TO_TARGET_1:
                    if (smoothDriveTo(TARGET_1)) {
                        telemetry.addLine("Reached Position #1!");
                        stateMachine = StateMachine.SLIDER_UP;
                    }
                    break;

                case SLIDER_UP:
                    moveSlider(130); // Move slider up
                    stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    break;

                case DRIVE_TO_TARGET_2:
                    if (smoothDriveTo(TARGET_2)) {
                        telemetry.addLine("Reached Position #2!");
                        stateMachine = StateMachine.SLIDER_DOWN_AND_OPEN_CLAW;
                    }
                    break;

                case SLIDER_DOWN_AND_OPEN_CLAW:
                    moveSliderAndOpenClaw(30);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    break;

                case DRIVE_TO_TARGET_3:
                    if (smoothDriveTo(TARGET_3)) {
                        telemetry.addLine("Reached Position #3!");
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;

                case AT_TARGET:
                    telemetry.addLine("Autonomous Complete!");
                    telemetry.update();
                    break;
            }

            telemetry.addData("State", stateMachine);
            telemetry.addData("Position", odo.getPosition().toString());
            telemetry.update();
        }
    }

    private boolean smoothDriveTo(Pose2D targetPose) {
        Pose2D currentPose = odo.getPosition();
        double xError = targetPose.getX(DistanceUnit.MM) - currentPose.getX(DistanceUnit.MM);
        double yError = targetPose.getY(DistanceUnit.MM) - currentPose.getY(DistanceUnit.MM);
        double distanceRemaining = Math.hypot(xError, yError);

        // Scale speed based on distance remaining
        double maxSpeed = 0.7;
        double minSpeed = 0.2;
        double speed = Math.max(minSpeed, Math.min(maxSpeed, distanceRemaining / 200.0)); // Scale between 0.2 and 0.7.
        // About the integer under distanceRemaining, 300.0 or 400.0 works well for
        // longer dutances (> 1000mm).

        boolean atTarget = nav.driveTo(currentPose, targetPose, speed, targetPose.getHeading(AngleUnit.DEGREES));

        // Apply calculated motor powers
        leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
        rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
        leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
        rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

        // Update telemetry
        telemetry.addData("State", "Driving to Target");
        telemetry.addData("Distance Remaining", "%.2f mm", distanceRemaining);
        telemetry.update();

        return atTarget;

    }

    private void moveSlider(int mm) {
        int ticks = mm * 10; // Conversion
        ySliderMotor.setTargetPosition(ySliderMotor.getCurrentPosition() + ticks);
        ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ySliderMotor.setPower(0.5);

        while (ySliderMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Y Slider Position", ySliderMotor.getCurrentPosition());
            telemetry.update();
        }
        ySliderMotor.setPower(0);
    }

    private void moveSliderAndOpenClaw(int mm) {
        int ticks = Math.abs(mm) * 10; // Convert mm to ticks
        int targetPosition = ySliderMotor.getCurrentPosition() - ticks;
        ySliderMotor.setTargetPosition(targetPosition);
        ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ySliderMotor.setPower(-0.4);

        boolean clawOpened = false;

        while (ySliderMotor.isBusy() && opModeIsActive()) {
            if (!clawOpened) {
                clawServo.setPosition(0.2); // Keep claw slightly closed initially
                clawOpened = true;
            }
        }
        ySliderMotor.setPower(0);
        clawServo.setPosition(0); // Fully open claw
    }
}
