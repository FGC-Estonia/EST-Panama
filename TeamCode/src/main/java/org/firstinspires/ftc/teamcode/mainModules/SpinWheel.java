package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SpinWheel {

    private Servo wheelServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public SpinWheel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }

    private void mapServo() {
        wheelServo = hardwareMap.get(Servo.class, "Servo_Port_0_CH");
        wheelServo.setDirection(Servo.Direction.FORWARD);
    }

    public void spin(boolean clockwise) {
        if (clockwise) {
            wheelServo.setPosition(1.0); // example max position (adjust as needed)
        } else {
            wheelServo.setPosition(0); // example min position (adjust as needed)
        }
    }

    public void stop() {
        wheelServo.setPosition(0.5);
    }
}
