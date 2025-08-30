package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class RaiseFlag {

    private Servo flagServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public RaiseFlag(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }

    private void mapServo() {
        flagServo = hardwareMap.get(Servo.class, HardwareConstants.FLAG_SERVO_MOTOR);
        flagServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setPos(double pos) {
        flagServo.setPosition(pos);
    }

}
