package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;

import static org.firstinspires.ftc.teamcode.mainModules.MoveRobotTank.DriveGear;

public class Autonomous {

    private enum State {
        SEARCHING, APPROACHING, BACKING_UP, IDLE
    }

    private State state = State.IDLE;
    private AprilTagDetection currentTag = null;
    private int[] startPos = null;
    private long startTime = 0;

    private static final double TICKS_PER_REV = 560.0;
    private static final double WHEEL_DIAMETER = 0.09; // m
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    /** Step function: call this each TeleOp loop while triangle is held */
    public void goToAprilTagAndBack(LinearOpMode op,
                                        MoveRobot moveRobot,
                                        AprilTagProcessor aprilTag,
                                        VisionPortal visionPortal) {
        if (state == State.IDLE) {
            state = State.SEARCHING;
        }

        switch (state) {
            case SEARCHING:
                List<AprilTagDetection> detections = aprilTag.getDetections();
                if (!detections.isEmpty()) {
                    currentTag = detections.get(0);
                    state = State.APPROACHING;
                    moveRobot.move(0, 0, 0, 0, false, DriveGear.LOW); // stop
                } else {
                    moveRobot.move(0, 0.2, 0, 0, false, DriveGear.LOW); // drive forward slowly
                }
                break;

            case APPROACHING:
                final double targetDistanceIn = 15.7;
                if (currentTag != null && currentTag.ftcPose.range > targetDistanceIn) {
                    moveRobot.move(0, 0.3, 0, 0, false, DriveGear.LOW);
                } else {
                    moveRobot.move(0, 0, 0, 0, false, DriveGear.LOW);
                    startPos = moveRobot.getEncoderPositions();
                    startTime = System.currentTimeMillis();
                    state = State.BACKING_UP;
                }
                break;

            case BACKING_UP:
                if (startPos == null) startPos = moveRobot.getEncoderPositions();
                int ticks = (int) (1.0 / WHEEL_CIRCUMFERENCE * TICKS_PER_REV);

                int[] curPos = moveRobot.getEncoderPositions();
                int delta = ((curPos[0] - startPos[0]) +
                        (curPos[1] - startPos[1]) +
                        (curPos[2] - startPos[2]) +
                        (curPos[3] - startPos[3])) / 4;

                if (Math.abs(delta) >= Math.abs(ticks) ||
                        System.currentTimeMillis() - startTime > 3000) {
                    moveRobot.move(0, 0, 0, 0, false, DriveGear.LOW);
                    state = State.IDLE;
                } else {
                    moveRobot.move(0, -0.4, 0, 0, false, DriveGear.MEDIUM);
                }
                break;

            case IDLE:
            default:
                moveRobot.move(0, 0, 0, 0, false, DriveGear.LOW);
                break;
        }
    }

    /** Cancel sequence and stop motors */
    public void reset(MoveRobot moveRobot) {
        state = State.IDLE;
        currentTag = null;
        startPos = null;
        moveRobot.move(0, 0, 0, 0, false, DriveGear.LOW);
    }
}
