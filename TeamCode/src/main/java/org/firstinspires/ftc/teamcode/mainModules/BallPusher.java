package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class BallPusher {

    private DcMotorEx motor = null;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public BallPusher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotors();
    }


    private void mapMotors() {
        motor = hardwareMap.get(DcMotorEx.class, HardwareConstants.BALL_PUSHER_MOTOR);

        motor.setDirection(DcMotor.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset encoder here
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void pushingBalls(int direction) {


        if (direction > 0) {
            extend(-650);
            motor.setPower(0);
        } else if (direction < 0) {
            extend(0);
            motor.setPower(0);
        }

        motor.setPower(direction);
        telemetry.addData("Distance", motor.getCurrentPosition());
    }

    public void extend(int distance) {
        motor.setTargetPosition(distance); //1000(height mm)/(6mm(hex shaft diameter)*3 ,14)*28(ticks per rotation)
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //runs to position

    }
}
