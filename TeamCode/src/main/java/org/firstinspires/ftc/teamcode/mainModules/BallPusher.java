package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        motor = hardwareMap.get(DcMotorEx.class, "Motor_Port_2_EH");

        motor.setDirection(DcMotor.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void pushingBalls(int direction) {
        double power;

        if (direction == 1) {  // Climb up
            power = 1.0;
        } else if (direction == -1) {  // Climb down slowly
            power = -1.0;
        } else {
            power = 0;
        }

        motor.setPower(power);
    }

}
