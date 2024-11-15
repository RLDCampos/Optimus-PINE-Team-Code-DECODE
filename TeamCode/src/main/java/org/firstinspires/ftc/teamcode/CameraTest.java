package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Camera Test", group="Test")
public class CameraTest extends LinearOpMode {
    private VisionPortal visionPortal;
    
    @Override
    public void runOpMode() {
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "webcam_1"))
            .enableLiveView(true)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // or try YUY2 if needed
            .build();

        telemetry.addData("Status", "Camera Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Camera Status", "Streaming...");
            telemetry.update();
            sleep(50);
        }
        visionPortal.close();
    }
}
