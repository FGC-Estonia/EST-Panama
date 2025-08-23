package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Two Motor Toggle Control")
public class ThrowSystem extends LinearOpMode {
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;

    private static final double DEADZONE = 0.05;

    // toggle state
    private boolean controlEnabled = false;
    private boolean lastXPressed = false;

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Press X to toggle joystick control ON/OFF");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Handle toggle with X button ---
            if (gamepad1.x && !lastXPressed) {
                controlEnabled = !controlEnabled; // flip state
            }
            lastXPressed = gamepad1.x;

            if (controlEnabled) {
                double leftY = -gamepad1.left_stick_y;
                double rightY = -gamepad1.right_stick_y;

                motor1.setPower(applyDeadzone(leftY));
                motor2.setPower(applyDeadzone(rightY));
            } else {
                motor1.setPower(0);
                motor2.setPower(0);
            }

            telemetry.addData("Control Enabled", controlEnabled);
            telemetry.addData("motor1 power", "%.2f", motor1.getPower());
            telemetry.addData("motor2 power", "%.2f", motor2.getPower());
            telemetry.update();
        }
    }

    private double applyDeadzone(double v) {
        if (Math.abs(v) < DEADZONE) return 0.0;
        return Math.max(-1.0, Math.min(1.0, v));
    }
}
