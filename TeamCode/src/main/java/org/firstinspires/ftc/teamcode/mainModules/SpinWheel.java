package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class SpinWheel {

    private DcMotorEx wheelMotor;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private boolean spinning = false;

    public SpinWheel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotor();
    }

    private void mapMotor() {
        wheelMotor = hardwareMap.get(DcMotorEx.class, HardwareConstants.WHEEL_MOTOR);
        wheelMotor.setDirection(DcMotorEx.Direction.FORWARD);
        wheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void spin(boolean clockwise) {
        if (clockwise && !spinning) {
            wheelMotor.setVelocity(100 * Math.PI / 3, AngleUnit.RADIANS);
            spinning = true;
        } else if (!clockwise && spinning) {
            wheelMotor.setVelocity(0);
            spinning = false;
        }
    }

    public double getVelocity() {
        return wheelMotor.getVelocity(AngleUnit.RADIANS);
    }

    public void stop() {
        wheelMotor.setVelocity(0);
        spinning = false;
    }
}
