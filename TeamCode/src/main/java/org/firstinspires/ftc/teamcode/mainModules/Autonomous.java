package org.firstinspires.ftc.teamcode.mainModules;

import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.TrajectorySequence;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Autonomous {
    private final MoveRobot drive;
    //private final PoseEstimator poseEstimator;
    private final Telemetry telemetry;

    public Autonomous(MoveRobot moveRobot /*, PoseEstimator poseEstimator */, Telemetry telemetry) {
        this.drive = moveRobot;
       // this.poseEstimator = poseEstimator;
        this.telemetry = telemetry;
    }

    public void goToRope(ClimbRope.RopeID ropeID) {
//        Pose2d targetPose = ClimbRope.getRopePose(ropeID);
        // TODO: Build trajectory using RoadRunner builder
        // Example:
        // TrajectorySequence traj = drive.trajectorySequenceBuilder(poseEstimator.getPoseEstimate())
        //         .lineTo(targetPose.vec())
        //         .build();
        // drive.followTrajectorySequence(traj);

        telemetry.addData("Going to rope:", ropeID);
        telemetry.update();
    }
}