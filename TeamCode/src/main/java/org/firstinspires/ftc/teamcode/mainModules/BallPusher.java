package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class BallPusher {

    private Servo leftMotor = null;
    private Servo rightMotor = null;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public BallPusher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotors();
    }


    private void mapMotors() {
        try {
            leftMotor = hardwareMap.get(Servo.class, HardwareConstants.BALL_PUSHER_MOTOR_LEFT);
            rightMotor = hardwareMap.get(Servo.class, HardwareConstants.BALL_PUSHER_MOTOR_RIGHT);

            leftMotor.setDirection(Servo.Direction.FORWARD);
            rightMotor.setDirection(Servo.Direction.REVERSE);
        } catch (Exception e) {
            e.printStackTrace(); // logs the error for debugging
        }
    }


    public void open(float time) throws InterruptedException {
        if (leftMotor == null) return;

        leftMotor.setPosition(1);
        rightMotor.setPosition(1);

        Thread.sleep((long)(time * 1000)); // convert seconds to ms

        leftMotor.setPosition(0.5);
        rightMotor.setPosition(0.5);
    }

    public void close(float time) throws InterruptedException {
        if (leftMotor == null) return;

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);

        Thread.sleep((long)(time * 1000)); // convert seconds to ms

        leftMotor.setPosition(0.5);
        rightMotor.setPosition(0.5);
    }

}
