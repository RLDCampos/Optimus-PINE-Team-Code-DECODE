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

    private DcMotor leftFront, leftRear, rightRear, rightFront;
    private GoBildaPinpointDriver pinpoint;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(1, 1, 1, 1); // kV, kA, and track width (adjust values for your robot)

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_back");
        rightRear = hardwareMap.get(DcMotor.class, "right_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");

        // Initialize Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU(); // Ensure Pinpoint is calibrated

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Optional: Adjust PID coefficients for precise control
        PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(10, 0, 0);
        PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

        // Use Pinpoint for localization
        setLocalizer(new PinpointLocalizer(pinpoint));
    }

    @Override
    public List<Double> getWheelPositions() {
        return null; // Use Pinpoint for localization; wheels aren't the primary odometry source
    }

    @Override
    public List<Double> getWheelVelocities() {
        return null; // Same as above
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return pinpoint.getHeading(); // Use Pinpoint heading
    }
}
