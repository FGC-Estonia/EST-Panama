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
    private double lastForward = 0;
    private double lastTurn = 0;
    private boolean driveStraightModeOn = false;
    private double driveStraightAngle = 0;
    private final double GYRO_CORRECTION_MAX_AMOUNT = 0.2;
    private final double GYRO_CORRECTION_MULTIPLIER = 2;
    private static final double DEADBAND = 0.05;
    private double lastTimeCalledDrive = System.nanoTime();
    private final double SLEW_STEP_FORWARD = 0.05;
    private final double SLEW_STEP_TURN = 0.05;
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

    /* Limits acceleration changes by at most slewStep, taking into consideration update delays
     * smoothing abrupt inputs into gradual, predictable motion.*/
    private double applySlewRate(double current, double target, double slewStep, double deltaTime) {
        double delta = target - current;
        // maximum change allowed this frame
        double maxStep = slewStep * deltaTime;

        if (Math.abs(delta) > maxStep) {
            delta = Math.signum(delta) * maxStep;
        }
        return current + delta;
    }

    private double clamp(double current, double min, double max) {
        return Math.max(min, Math.min(current, max));
    }

    public void drive(double heading, double rawForward, double rawTurn, boolean speed1, boolean speed2, boolean speed3) {

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
        double now = System.nanoTime();
        double deltaTime = (now - lastTimeCalledDrive) / 1_000_000_000.0; // to seconds
        lastTimeCalledDrive = now;

        double f_slew = applySlewRate(lastForward, f_cu, SLEW_STEP_FORWARD, deltaTime);
        double t_slew = applySlewRate(lastTurn, t_cu, SLEW_STEP_TURN, deltaTime);

        lastForward = f_slew;
        lastTurn = t_slew;

        //final
        double leftPower = f_slew + t_slew;
        double rightPower = f_slew - t_slew;

        //drive forward using gyro
        if (t_slew == 0) {
            if (!driveStraightModeOn) {
                driveStraightModeOn = true;
                driveStraightAngle = heading;
            }

            double angleError = heading - driveStraightAngle;
            double correction = clamp(angleError * GYRO_CORRECTION_MULTIPLIER, -GYRO_CORRECTION_MAX_AMOUNT, GYRO_CORRECTION_MAX_AMOUNT);

            leftPower -= correction;
            rightPower += correction;
        } else {
            driveStraightModeOn = false;
        }


        //final input clamp to -1 -> 1
        leftPower = clamp(leftPower, -1, 1);
        rightPower = clamp(rightPower, -1, 1);

        if (useVelocity) {
            leftDrive.setVelocity(leftPower * MAX_VELOCITY);
            rightDrive.setVelocity(rightPower * MAX_VELOCITY);
        } else {
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.update();
    }
}