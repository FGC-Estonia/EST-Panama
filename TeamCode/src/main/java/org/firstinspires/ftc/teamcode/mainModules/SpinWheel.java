package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class SpinWheel {

    private DcMotorEx wheelMotor;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public SpinWheel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotor();
    }

    private void mapMotor() {
        wheelMotor = hardwareMap.get(DcMotorEx.class, HardwareConstants.WHEEL_MOTOR);
        wheelMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void spin(boolean clockwise) {
        if (clockwise) {
            wheelMotor.setVelocity(100*Math.PI/3, AngleUnit.RADIANS); // example max position (adjust as needed)
        } else {
            wheelMotor.setVelocity(0); // example min position (adjust as needed)
        }
    }

    public void stop() {
        wheelMotor.setVelocity(0);
    }
}
