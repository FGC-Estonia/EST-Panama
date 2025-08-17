package org.firstinspires.ftc.teamcode.mainModules;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
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
        aprilTag = new AprilTagProcessor.Builder()
                // optionally .setTagFamily(...), .setOutputUnits(...), etc.
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (useWebcam) {
            builder.setCamera(hardwareMap.get(WebcamName.class, webcamName));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // optional: builder.setCameraResolution(new Size(640, 480));
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
        for (AprilTagDetection d : det) {
            if (d.metadata != null) {
                telemetry.addLine(String.format(Locale.US,
                        "ID %d: %s", d.id, d.metadata.name));
                telemetry.addLine(String.format(Locale.US,
                        "XYZ %.1f, %.1f, %.1f (inch)",
                        d.ftcPose.x, d.ftcPose.y, d.ftcPose.z));
            } else {
                telemetry.addLine(String.format(Locale.US,
                        "ID %d (unknown) center %.1f, %.1f",
                        d.id, d.center.x, d.center.y));
            }
        }
    }
}
