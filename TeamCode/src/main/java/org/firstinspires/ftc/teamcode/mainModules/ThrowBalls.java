package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ThrowBalls {
    private final DcMotor motor1;
    private final DcMotor motor2;
    private final Telemetry telemetry;

    private static final double DEADZONE = 0.05;

    // control state (mirrors the toggle behavior from your TeleOp example)
    private boolean controlEnabled = false;

    // requested powers are set by the caller (range -1..1). update() applies them when enabled.
    private double requestedLeftPower = 0.0;
    private double requestedRightPower = 0.0;

    /**
     * Default constructor uses hardware names "motor1" and "motor2".
     */


    /**
     * Constructor which accepts custom motor names.
     */
    public ThrowBalls(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor1 = hardwareMap.get(DcMotor.class, "Motor_Port_1_CH");
        motor2 = hardwareMap.get(DcMotor.class, "Motor_Port_2_CH");

        // mirror settings from your example
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void start() {
        motor1.setPower(1.0);
        motor2.setPower(1.0);
    }
    public void stop() {
        motor1.setPower(0.0);
        motor2.setPower(0.0);
    }

}
