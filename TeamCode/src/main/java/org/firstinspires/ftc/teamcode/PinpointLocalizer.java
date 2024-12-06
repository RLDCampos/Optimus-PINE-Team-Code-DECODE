package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver pinpoint;

    public PinpointLocalizer(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    @Override
    public Pose2d getPoseEstimate() {
        // Convert Pinpoint's Pose2D to Road Runner's Pose2d
        Pose2D pose = pinpoint.getPosition();
        return new Pose2d(pose.getX(DistanceUnit.MM) / 1000.0,  // Convert mm to meters
                pose.getY(DistanceUnit.MM) / 1000.0,  // Convert mm to meters
                pose.getHeading());
    }

    @Override
    public void setPoseEstimate(Pose2d pose2d) {
        // Update Pinpoint's position to match Road Runner's estimate
        pinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.MM,
                pose2d.getX() * 1000,  // Convert meters to mm
                pose2d.getY() * 1000,  // Convert meters to mm
                pose2d.getHeading()
        ));
    }

    @Override
    public Pose2d getPoseVelocity() {
        // Return velocity if available from Pinpoint
        Pose2D velocity = pinpoint.getVelocity();
        return new Pose2d(velocity.getX(DistanceUnit.MM) / 1000.0,
                velocity.getY(DistanceUnit.MM) / 1000.0,
                velocity.getHeading());
    }
}
