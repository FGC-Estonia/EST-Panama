package org.firstinspires.ftc.teamcode.mainModules;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

public class AprilTagManager {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public AprilTagManager(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    /** Call once before using getDetections/sendTelemetry. */
    public void init(boolean useWebcam, String webcamName) {
        // Use the easy convenience factory which includes the default tag library
        // (this populates metadata for tags that are in the default library).
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()) // or your custom library
                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (useWebcam) {
            builder.setCamera(hardwareMap.get(WebcamName.class, webcamName));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Pose estimation requires camera intrinsics for the chosen resolution.
        // Set an explicit resolution that matches your camera / calibration if possible.
        builder.setCameraResolution(new Size(640, 480));

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    /** Safe getter for detections (never null). */
    public List<AprilTagDetection> getDetections() {
        if (aprilTag == null) return Collections.emptyList();
        return aprilTag.getDetections();
    }

    public void resumeStreaming() {
        if (visionPortal != null) visionPortal.resumeStreaming();
    }

    public void stopStreaming() {
        if (visionPortal != null) visionPortal.stopStreaming();
    }

    public void close() {
        if (visionPortal != null) visionPortal.close();
    }

    /** Convenience: copy of the sample's telemetry formatting (trimmed). */
    public void sendTelemetry() {
        List<AprilTagDetection> det = getDetections();
        telemetry.addData("# AprilTags Detected", det.size());
        for (AprilTagDetection detection : det) {
            // show ID always
            telemetry.addLine(String.format(Locale.US, "ID %d", detection.id));

            // If metadata present, show the friendly name
            if (detection.metadata != null) {
                telemetry.addLine(String.format(Locale.US, "Name: %s", detection.metadata.name));
            }

            // ftcPose may still be null even when metadata exists (depends on camera intrinsics / resolution).
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                // Fallback: show pixel center when pose isn't available
                telemetry.addLine(String.format(Locale.US,
                        "Center px %.1f, %.1f (no pose available)", detection.center.x, detection.center.y));
            }
        }
    }
}
