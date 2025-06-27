package org.firstinspires.ftc.teamcode.mainModules;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class ClimbRope {
    public enum RopeID {
        RED_LEFT, RED_CENTER, RED_RIGHT,
        BLUE_LEFT, BLUE_CENTER, BLUE_RIGHT
    }

    public static Pose2d getRopePose(RopeID id) {
        switch (id) {
            case RED_LEFT:
                return new Pose2d(1.0, 6.0, Math.toRadians(180));
            case RED_CENTER:
                return new Pose2d(3.5, 6.0, Math.toRadians(180));
            case RED_RIGHT:
                return new Pose2d(6.0, 6.0, Math.toRadians(180));
            case BLUE_LEFT:
                return new Pose2d(1.0, 1.0, Math.toRadians(0));
            case BLUE_CENTER:
                return new Pose2d(3.5, 1.0, Math.toRadians(0));
            case BLUE_RIGHT:
                return new Pose2d(6.0, 1.0, Math.toRadians(0));
            default:
                return new Pose2d();
        }
    }

    public void startClimb() {
        // TODO: Activate motor to pull up
    }
}