package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MoveRobotTank {

    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final boolean useVelocity;
    private final boolean protect;
    private double maxSpeed = 1.0;
    private double lastLeftPower = 0;
    private double lastRightPower = 0;
    private static final double DEADBAND = 0.05;
    private final double SLEW_STEP = 0.05;
    private final double MAX_VELOCITY = 1972.92;

    public MoveRobotTank(boolean protect, HardwareMap hardwareMap, Telemetry telemetry, boolean useVelocity) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.useVelocity = useVelocity;
        mapMotors();
    }

    private void mapMotors() {
        leftDrive = hardwareMap.get(DcMotorEx.class, "Motor_Port_2_CH");
        rightDrive = hardwareMap.get(DcMotorEx.class, "Motor_Port_3_CH");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        if (useVelocity) {
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //ignore, when joystick has moved very slightly
    private double applyDeadband(double input, double deadband) {
        if (Math.abs(input) < deadband) return 0.0;
        return Math.copySign((Math.abs(input) - deadband) / (1.0 - deadband), input);
    }

    private double cubicScaling(double input) {
        return Math.pow(input, 3);
    }

    private double applySlewRate(double current, double target) {
        double delta = target - current;
        if (Math.abs(delta) > SLEW_STEP) {
            delta = Math.signum(delta) * SLEW_STEP;
        }
        return current + delta;
    }

    private double clamp(double current, double min, double max) {
        return Math.max(min, Math.min(current, max));
    }

    public void drive(double rawForward, double rawTurn, boolean speed1, boolean speed2, boolean speed3) {

        if (speed1) {
            maxSpeed = 0.25;
            telemetry.addData("Gear", "Low");
        } else if (speed2) {
            maxSpeed = 0.5;
            telemetry.addData("Gear", "Medium");
        } else if (speed3) {
            maxSpeed = 1.0;
            telemetry.addData("Gear", "High");
        }

        //input processing
        //deadband
        double f_db = applyDeadband(rawForward,  DEADBAND);
        double t_db = applyDeadband(rawTurn,     DEADBAND);

        //Cubic scaling
        double f_cu = cubicScaling(f_db) * maxSpeed;
        double t_cu = cubicScaling(t_db) * maxSpeed;

        //slew
        double f_slew = applySlewRate(lastLeftPower, f_cu);
        double t_slew = applySlewRate(lastRightPower, t_cu);

        //final
        lastLeftPower = f_slew + t_slew;
        lastRightPower = f_slew - t_slew;

        //final input clamp to -1 -> 1
        lastLeftPower = clamp(lastLeftPower, -1, 1);
        lastRightPower = clamp(lastRightPower, -1, 1);

        if (useVelocity) {
            leftDrive.setVelocity(lastLeftPower * MAX_VELOCITY);
            rightDrive.setVelocity(lastRightPower * MAX_VELOCITY);
        } else {
            leftDrive.setPower(lastLeftPower);
            rightDrive.setPower(lastRightPower);
        }

        telemetry.addData("Left Power", lastLeftPower);
        telemetry.addData("Right Power", lastRightPower);
        telemetry.update();
    }
}