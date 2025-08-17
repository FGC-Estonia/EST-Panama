package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.DriveBaseController;
import org.firstinspires.ftc.teamcode.common.util.SlewRateLimiter;



public class MoveRobotTank implements DriveBaseController {
    // This class is responsible for the movement of a robot with a tank drivebase

    // Declaration of variables
    // --- Hardware Objects ---
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;

    // --- System References ---
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // --- Configuration Flags ---
    private final boolean useVelocity;
    private final boolean protect; // Currently unused, consider removing or implementing.

    // --- Drive Constants ---
    private static final double MAX_TURN_SPEED = 0.5;
    private static final double MAX_TURN_DURING_CURVE = 0.2;
    private static final double DRIVE_DEADZONE = 0.05; // Joystick deadband
    private static final double TURN_DEADZONE = 0.02; // Joystick deadband
    private static final double STATIONARY_TURN_THRESHOLD = 0.05;
    private static final double CURVATURE_DRIVE_FACTOR = 0.8;
    private static final double MAX_MOTOR_VELOCITY_TPS = 1972.92;

    // --- Slew Rate Limiter Constants ---
    private static final double DRIVER_SLEW_RATE = 4.0;
    private static final double TURN_SLEW_RATE = 20.0;
    private static final double GYRO_SLEW_RATE = 4.0;

    // --- Slew Rate Limiter Instances ---
    private SlewRateLimiter driverLimiter;
    private SlewRateLimiter turnLimiter;
    private SlewRateLimiter gyroLimiter;

    // --- Heading Hold Variables ---
    private double wantedHeading = 0;
    private boolean headingHoldEnabled = false;
    private final double HEADING_KP = 0.25; // Tunable: radians -> motor power (renamed to constant)

    // --- Current State Variables (Dynamic) ---
    private double maxSpeed = 1.0; // Dynamic, set by DriveGear
    private double turnSpeed = 0.8; // Dynamic, set by DriveGear
    private double lastLeftPower = 0;
    private double lastRightPower = 0;
    private double lastDrive = 0;
    private double lastTurn = 0;


     // Defines the different drive speed gears.

    public enum DriveGear {
        LOW(0.35, 0.4, "Low"),
        MEDIUM(0.6, 0.5, "Medium"),
        HIGH(1.0, 0.8, "High");

        public final double maxSpeed;
        public final double turnSpeed;
        public final String telemetryName;

        DriveGear(double maxSpeed, double turnSpeed, String telemetryName) {
            this.maxSpeed = maxSpeed;
            this.turnSpeed = turnSpeed;
            this.telemetryName = telemetryName;
        }
    }
    public MoveRobotTank(boolean protect, HardwareMap hardwareMap, Telemetry telemetry, boolean useVelocity) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.useVelocity = useVelocity;
        mapMotors();
    }

    private void mapMotors() {
        leftDrive = hardwareMap.get(DcMotorEx.class, HardwareConstants.LEFT_BACK_MOTOR);
        rightDrive = hardwareMap.get(DcMotorEx.class, HardwareConstants.RIGHT_BACK_MOTOR);

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driverLimiter = new SlewRateLimiter(DRIVER_SLEW_RATE);
        turnLimiter = new SlewRateLimiter(TURN_SLEW_RATE);
        gyroLimiter = new SlewRateLimiter(GYRO_SLEW_RATE);

        // Assesses whether or not the motor should run using velocity or power
        if (useVelocity) {
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // Ignores input when the joystick moves very slightly
    private double applyDeadzone(double input, double deadzone) {
        if (Math.abs(input) < deadzone) return 0.0;
        return Math.copySign((Math.abs(input) - deadzone) / (1.0 - deadzone), input);
    }

    @Override
    public void drive(double imuAngle,
                      double imuPitch,
                      double frontBack,
                      double leftRight,
                      double turnInput,
                      boolean fieldCentric,
                      DriveGear gear) {

        // Tank drive ignores strafe (leftRight) and fieldCentric
        this.maxSpeed = gear.maxSpeed;
        this.turnSpeed = gear.turnSpeed;
        telemetry.addData("Gear", gear.telemetryName);

        // before limiting:
        double rawDrive = applyDeadzone(frontBack, DRIVE_DEADZONE);
        double rawTurn  = applyDeadzone(turnInput, TURN_DEADZONE) * turnSpeed;

        // --- Counterstrife logic for drive and turn ---
        boolean reversingDrive = (Math.signum(rawDrive) != Math.signum(lastDrive)) && lastDrive != 0 && (Math.abs(rawDrive) > 0.05);
        boolean reversingTurn  = (Math.signum(rawTurn)  != Math.signum(lastTurn)) && lastTurn != 0 && (Math.abs(rawTurn) > 0.05);

        // cubic scaling:
        double drive = driverLimiter.calculate(Math.pow(rawDrive, 3));
        boolean stationaryTurn = Math.abs(drive) < STATIONARY_TURN_THRESHOLD;

        double rawTurnCubed = Math.pow(rawTurn, 3);
        double turn = stationaryTurn ? rawTurnCubed : rawTurnCubed;
        turn = Math.copySign(Math.min(Math.abs(turn), MAX_TURN_SPEED), turn);

        drive = (rawDrive == 0) ? 0 : drive;
        turn  = (rawTurn == 0) ? 0 : turn;

        lastDrive = drive;
        lastTurn  = turn;

        telemetry.addData("raw drive", frontBack);
        telemetry.addData("deadzone drive", rawDrive);
        telemetry.addData("final drive input", drive);

        double leftTargetPower, rightTargetPower;

        if (stationaryTurn) {
            // on-the-spot pivot
            leftTargetPower  = drive + turn * Math.pow(turn / MAX_TURN_SPEED, 2);
            rightTargetPower = drive - turn * Math.pow(turn / MAX_TURN_SPEED, 2);
        } else {
            // smooth curvature drive
            leftTargetPower  = drive + turn;
            rightTargetPower = drive - turn;

            double maxAbsPower = Math.max(Math.abs(leftTargetPower), Math.abs(rightTargetPower));
            if (maxAbsPower > 1.0) {
                leftTargetPower /= maxAbsPower;
                rightTargetPower /= maxAbsPower;
            }
        }

        telemetry.addData("use stationaryTurn ", stationaryTurn);

        // Heading hold
        double avgInput = (leftTargetPower + rightTargetPower) / 2;
        double diff = Math.abs(leftTargetPower - rightTargetPower);

        if (diff < 0.05 && Math.abs(avgInput) > 0.05) {
            if (!headingHoldEnabled) {
                wantedHeading = imuAngle; // use imuAngle now
            }
            headingHoldEnabled = true;
        } else {
            headingHoldEnabled = false;
        }

        if (headingHoldEnabled) {
            double headingError = normalizeRadians(wantedHeading - imuAngle);
            double rawCorrection = headingError * HEADING_KP;
            double correction = gyroLimiter.calculate(rawCorrection);
            leftTargetPower  -= correction;
            rightTargetPower += correction;
        }

        // Apply gear scaling
        leftTargetPower  *= maxSpeed;
        rightTargetPower *= maxSpeed;

        lastLeftPower  = Range.clip(leftTargetPower,  -1, 1);
        lastRightPower = Range.clip(rightTargetPower, -1, 1);

        // --- Auto-brake when sticks are released but wheels still spinning ---
        if (rawDrive == 0 && rawTurn == 0) {
            double leftVel  = leftDrive.getVelocity();
            double rightVel = rightDrive.getVelocity();

            double stopThreshold = 80.0;
            if (Math.abs(leftVel) > stopThreshold || Math.abs(rightVel) > stopThreshold) {
                double brakePowerLeft  = -Math.signum(leftVel)  * 0.2;
                double brakePowerRight = -Math.signum(rightVel) * 0.2;

                leftDrive.setPower(brakePowerLeft);
                rightDrive.setPower(brakePowerRight);
                return;
            }
            lastLeftPower = 0;
            lastRightPower = 0;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            return;
        }

        if (useVelocity) {
            leftDrive.setVelocity(lastLeftPower * MAX_MOTOR_VELOCITY_TPS);
            rightDrive.setVelocity(lastRightPower * MAX_MOTOR_VELOCITY_TPS);
        } else {
            leftDrive.setPower(lastLeftPower);
            rightDrive.setPower(lastRightPower);
        }

        telemetry.addData("Left Power", lastLeftPower);
        telemetry.addData("Right Power", lastRightPower);
        telemetry.addData("Left Velocity", leftDrive.getVelocity());
        telemetry.addData("Right Velocity", rightDrive.getVelocity());
        telemetry.addData("Heading Error", normalizeRadians(wantedHeading - imuAngle));
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