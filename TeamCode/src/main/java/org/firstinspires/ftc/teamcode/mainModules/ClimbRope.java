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
    private final double CLIMB_DOWN_RATE = 0.4;
    private final double STAY_ON_RATE = 0.1;
    boolean resettingPosition = false;

    public ClimbRope(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotors();
    }

    private void mapMotors() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "Motor_Port_0_EH");
        rightMotor = hardwareMap.get(DcMotorEx.class, "Motor_Port_1_EH");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    // ROTATING TO POS DOESNT REALLY WORK
    public void rotateToPosition(int targetTicks) {
        /*resettingPosition = true;
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setTargetPosition(targetTicks);
        rightMotor.setTargetPosition(targetTicks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(1.0);
        rightMotor.setPower(1.0);

        // Wait until both motors reach target
        while (leftMotor.isBusy() || rightMotor.isBusy()) {
            telemetry.addData("Left Pos", leftMotor.getCurrentPosition());
            telemetry.addData("Right Pos", rightMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resettingPosition = false;*/
    }


    public void ropeClimbing(int direction) {
        if (resettingPosition) {return;}

        double leftPower, rightPower;

        if (direction == 2) {  // Climb up
            leftPower = 1.0;
            rightPower = 1.0;
        } else if (direction == -1) {  // Climb down slowly
            leftPower = -CLIMB_DOWN_RATE;
            rightPower = -CLIMB_DOWN_RATE;
        } else if (direction == 1) {
            leftPower = STAY_ON_RATE;
            rightPower = STAY_ON_RATE;
        } else {
            leftPower = 0;
            rightPower = 0;
        }

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
}