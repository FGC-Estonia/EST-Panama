package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.SlewRateLimiter;

public class MoveRobotTank {

    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final boolean useVelocity;
    private final boolean protect;
    private double maxSpeed = 1.0;
    private double turnSpeed = 0.9;
    private double MAX_TURN_SPEED = 0.5;
    private double MAX_TURN_DURING_CURVE = 0.2;
    private double lastLeftPower = 0;
    private double lastRightPower = 0;
    private final double MAX_VELOCITY = 1972.92;
    private SlewRateLimiter driverLimiter;
    private SlewRateLimiter turnLimiter;
    private SlewRateLimiter gyroLimiter;

    private double wantedHeading = 0;
    private boolean headingHoldEnabled = false;
    private final double headingKp = 0.25; // Tunable: radians -> motor power

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

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driverLimiter = new SlewRateLimiter(4);
        turnLimiter = new SlewRateLimiter(20);
        gyroLimiter = new SlewRateLimiter(4);


        if (useVelocity) {
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //ignore, when joystick has moved very slightly
    private double applyDeadzone(double input, double deadband) {
        if (Math.abs(input) < deadband) return 0.0;
        return Math.copySign((Math.abs(input) - deadband) / (1.0 - deadband), input);
    }




    public void drive(double currentHeading, double currentPitch, double driveInput, double turnInput,
                      boolean speed1, boolean speed2, boolean speed3) {

        if (speed1) {
            maxSpeed = 0.35;
            turnSpeed = 0.7;
            telemetry.addData("Gear", "Low");
        } else if (speed2) {
            maxSpeed = 0.6;
            turnSpeed = 0.8;
            telemetry.addData("Gear", "Medium");
        } else if (speed3) {
            maxSpeed = 1.0;
            turnSpeed = 0.9;
            telemetry.addData("Gear", "High");
        }



        // before limiting:
        double rawDrive = applyDeadzone(driveInput, 0.05);
        double rawTurn  = applyDeadzone(turnInput, 0.05) * turnSpeed;

        // cubic scaling:
        double drive = driverLimiter.calculate(Math.pow(rawDrive, 3));
        boolean quickTurn = Math.abs(drive) == 0;

        double rawTurnCubed = Math.pow(rawTurn, 3);
        double turn;
        if (quickTurn) {
            // super‑snappy but still protected
            turn = turnLimiter.calculate(rawTurnCubed);
        } else {
            // gentle curvature
            turn = rawTurnCubed;
        }

        turn = Math.min(turn, MAX_TURN_SPEED);

        drive = (rawDrive == 0) ? 0 : drive;
        turn = (rawTurn == 0) ? 0 : turn;

        telemetry.addData("raw drive", driveInput);
        telemetry.addData("deadzone drive", rawDrive);
        telemetry.addData("final drive input", drive);


        double leftTarget, rightTarget;
        if (quickTurn) {
            // on‑the‑spot pivot
            leftTarget  = drive + turn;
            rightTarget = drive - turn;
        } else {  //TO DO CURVATURE IS BUGGED. SOMEHOW REVERSED CONTROLS WITH IT
            // smooth curvature drive
            turn = Math.min(MAX_TURN_DURING_CURVE, turn);
            leftTarget  = drive - turn * Math.abs(drive) * 0.8;
            rightTarget = drive + turn * Math.abs(drive) * 0.8;
        }

        telemetry.addData("use quickturn ", quickTurn);


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

        //move forward only
        if (headingHoldEnabled) {
            double headingError = normalizeRadians(wantedHeading - currentHeading);
            double rawCorrection = headingError * headingKp;
            double correction = gyroLimiter.calculate(rawCorrection);
            leftTarget -= correction;
            rightTarget += correction;
        }

        leftTarget *= maxSpeed;
        rightTarget *= maxSpeed;

        //clip to range (-1 -> 1)
        lastLeftPower  = Range.clip(leftTarget,  -1, 1);
        lastRightPower = Range.clip(rightTarget, -1, 1);




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

    public int[] getEncoderPositions() {
        return new int[]{
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition()
        };
    }
}