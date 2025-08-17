package org.firstinspires.ftc.teamcode.mainModules;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Autonomous {
    private final MoveRobot drive;
    private final Telemetry telemetry;
    private final AprilTagManager aprilTagManager;   // declare only

    // Accept HardwareMap here so we can create the AprilTag helper
    public Autonomous(MoveRobot moveRobot, HardwareMap hardwareMap, Telemetry telemetry) {
        this.drive = moveRobot;
        this.telemetry = telemetry;

        // create & init april after telemetry/hardwareMap exist
        aprilTagManager = new AprilTagManager(hardwareMap, telemetry);
        aprilTagManager.init(true, "Webcam 1");
    }

    public void showAprilTagData() {
        // example: helper posts telemetry like the sample
        aprilTagManager.sendTelemetry();
    }

    public void stopApril() {
        aprilTagManager.close();
    }
}
