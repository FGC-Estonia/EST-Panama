package org.firstinspires.ftc.teamcode.mainModules;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClimbRope {
    private DcMotorEx leftMotor = null;
    private DcMotorEx rightMotor = null;
    private final boolean protect;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public ClimbRope(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotors();
    }

    private void mapMotors() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "Motor_Port_4_CH");
        rightMotor = hardwareMap.get(DcMotorEx.class, "Motor_Port_5_CH");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

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
            //    return new Pose2d(); //
        }
        return null;
    }

    public void startClimb(boolean goUp) {
        int direction = goUp ? 1 : -1;

        leftMotor.setPower(direction);
        rightMotor.setPower(direction);
    }

    public void endClimb() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}