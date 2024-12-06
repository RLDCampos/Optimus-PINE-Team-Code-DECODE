package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

public class SampleMecanumDrive extends MecanumDrive {

    // Motors
    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightFront;
    private final DcMotor rightBack;

    // goBILDA Pinpoint Odometry Computer
    private final GoBildaPinpointDriver pinpoint;

    // PID coefficients (adjust as needed)
    private static final PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(10, 0, 0);
    private static final PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(1, 1, 1, 1); // Set appropriate track width, kV, kA, and angular kV for your robot

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        // Initialize Pinpoint odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU(); // Reset IMU and position for accurate starting coordinates

        // Configure motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Configure braking behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Use the Pinpoint as the localizer
        setLocalizer(new PinpointLocalizer(pinpoint));
    }

    @Override
    public List<Double> getWheelPositions() {
        // Pinpoint handles localization, so no need to return wheel positions
        return null;
    }

    @Override
    public List<Double> getWheelVelocities() {
        // Pinpoint handles localization, so no need to return wheel velocities
        return null;
    }

    @Override
    public void setMotorPowers(double frontLeft, double backLeft, double backRight, double frontRight) {
        // Set motor powers for Mecanum drive
        leftFront.setPower(frontLeft);
        leftBack.setPower(backLeft);
        rightBack.setPower(backRight);
        rightFront.setPower(frontRight);
    }

    @Override
    public double getRawExternalHeading() {
        // Use Pinpoint's heading
        return pinpoint.getHeading();
    }
}
