/*
package org.firstinspires.ftc.teamcode.mainModules;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PoseEstimator {
    private final ImuManager imu;
    private Pose2d currentPose = new Pose2d();

    public PoseEstimator(ImuManager imuManager) {
        this.imu = imuManager;
    }

    public void update(double deltaLeft, double deltaRight, double deltaStrafe) {
        double heading = imu.getYawRadians();
        double dx = deltaLeft; // TODO: Better fusion
        double dy = deltaStrafe;
        currentPose = new Pose2d(
                currentPose.getX() + dx,
                currentPose.getY() + dy,
                heading
        );
    }

    public Pose2d getPoseEstimate() {
        return currentPose;
    }

    public void setPoseEstimate(Pose2d pose) {
        this.currentPose = pose;
    }
}

 */