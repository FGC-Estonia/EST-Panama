package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

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
    private final double SLEW_STEP = 0.05;
    private final double MAX_VELOCITY = 1972.92;

    private double wantedHeading = 0;
    private boolean headingHoldEnabled = false;
    private final double headingKp = 0.6; // Tunable: radians -> motor power

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

    public void drive(double currentHeading, double currentPitch, double driveInput, double turnInput,
                      boolean speed1, boolean speed2, boolean speed3,
                      boolean holdPitch, double targetPitchRad) {

        if (speed1) {
            maxSpeed = 0.35;
            telemetry.addData("Gear", "Low");
        } else if (speed2) {
            maxSpeed = 0.6;
            telemetry.addData("Gear", "Medium");
        } else if (speed3) {
            maxSpeed = 1.0;
            telemetry.addData("Gear", "High");
        }

        double drive = cubicScaling(driveInput);
        double turn = cubicScaling(turnInput);

        double leftTarget = drive + turn;
        double rightTarget = drive - turn;

        double avgInput = (leftTarget + rightTarget) / 2;
        double diff = Math.abs(leftTarget - rightTarget);

        if (diff < 0.05 && Math.abs(avgInput) > 0.05) {
            if (!headingHoldEnabled) {
                wantedHeading = currentHeading;
            }
            headingHoldEnabled = true;
        } else {
            headingHoldEnabled = false;
        }

        if (headingHoldEnabled) {
            double headingError = normalizeRadians(wantedHeading - currentHeading);
            double correction = headingError * headingKp;
            leftTarget -= correction;
            rightTarget += correction;
        }

        leftTarget *= maxSpeed;
        rightTarget *= maxSpeed;

        if (holdPitch) {
            double pitchError = targetPitchRad - currentPitch;
            double kP_pitch = 4.0;  // Tune this gain
            double basePower = 0.4;
            double pitchCorrection = pitchError * kP_pitch;
            double wheeliePower = Range.clip(basePower + pitchCorrection, -1.0, 1.0);

            leftTarget = wheeliePower + turn;
            rightTarget = wheeliePower - turn;
        }

        if (holdPitch || headingHoldEnabled) {
            lastLeftPower = leftTarget;
            lastRightPower = rightTarget;
        } else {
            lastLeftPower = applySlewRate(lastLeftPower, leftTarget);
            lastRightPower = applySlewRate(lastRightPower, rightTarget);
        }


        if (useVelocity) {
            leftDrive.setVelocity(lastLeftPower * MAX_VELOCITY);
            rightDrive.setVelocity(lastRightPower * MAX_VELOCITY);
        } else {
            leftDrive.setPower(lastLeftPower);
            rightDrive.setPower(lastRightPower);
        }

        telemetry.addData("Left Power", lastLeftPower);
        telemetry.addData("Right Power", lastRightPower);
        telemetry.addData("Left Velocity", leftDrive.getVelocity());
        telemetry.addData("Right Velocity", rightDrive.getVelocity());
        telemetry.addData("Heading Error", normalizeRadians(wantedHeading - currentHeading));
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}