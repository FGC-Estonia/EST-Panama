package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.SlewRateLimiter;



public class MoveRobotTank {
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
    private static final double MOTOR_DEADZONE = 0.05; // Joystick deadband
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
    private double turnSpeed = 0.9; // Dynamic, set by DriveGear
    private double lastLeftPower = 0;
    private double lastRightPower = 0;


     // Defines the different drive speed gears.

    public enum DriveGear {
        LOW(0.35, 0.7, "Low"),
        MEDIUM(0.6, 0.8, "Medium"),
        HIGH(1.0, 0.9, "High");

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
        leftDrive = hardwareMap.get(DcMotorEx.class, "Motor_Port_2_CH");
        rightDrive = hardwareMap.get(DcMotorEx.class, "Motor_Port_3_CH");

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
    private double applyDeadzone(double input, double deadband) {
        if (Math.abs(input) < deadband) return 0.0;
        return Math.copySign((Math.abs(input) - deadband) / (1.0 - deadband), input);
    }
    
    public void drive(double currentHeading, double currentPitch, double driveInput, double turnInput,
                      DriveGear requestedGear) {

        // Set speed parameters based on the requested gear
        this.maxSpeed = requestedGear.maxSpeed;
        this.turnSpeed = requestedGear.turnSpeed;
        telemetry.addData("Gear", requestedGear.telemetryName);



        // before limiting:
        double rawDrive = applyDeadzone(driveInput, MOTOR_DEADZONE);
        double rawTurn  = applyDeadzone(turnInput, MOTOR_DEADZONE) * turnSpeed;

        // cubic scaling:
        double drive = driverLimiter.calculate(Math.pow(rawDrive, 3));
        boolean stationaryTurn = Math.abs(drive) < STATIONARY_TURN_THRESHOLD;

        double rawTurnCubed = Math.pow(rawTurn, 3);
        double turn;
        if (stationaryTurn) {
            // super‑snappy but still protected
            turn = rawTurnCubed;
        } else {
            // gentle curvature
            turn = turnLimiter.calculate(rawTurnCubed);
        }

        turn = Math.min(turn, MAX_TURN_SPEED);

        drive = (rawDrive == 0) ? 0 : drive;
        turn = (rawTurn == 0) ? 0 : turn;

        telemetry.addData("raw drive", driveInput);
        telemetry.addData("deadzone drive", rawDrive);
        telemetry.addData("final drive input", drive);


        double leftTargetPower, rightTargetPower;
        if (stationaryTurn) {
            // on‑the‑spot pivot
            leftTargetPower  = drive + turn;
            rightTargetPower = drive - turn;
        } else {  //TO DO CURVATURE IS BUGGED. SOMEHOW REVERSED CONTROLS WITH IT -- Possibly fixed, haven't tested
            // smooth curvature drive

            leftTargetPower  = drive + turn * Math.abs(drive);
            rightTargetPower = drive - turn * Math.abs(drive);
        }

        telemetry.addData("use stationaryTurn ", stationaryTurn);

        // Calculates and checks conditions for holding heading
        double avgInput = (leftTargetPower + rightTargetPower) / 2;
        double diff = Math.abs(leftTargetPower - rightTargetPower);

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
            double rawCorrection = headingError * HEADING_KP;
            double correction = gyroLimiter.calculate(rawCorrection);
            leftTargetPower -= correction;
            rightTargetPower += correction;
        }

        // Applies maxSpeed to the calculated target powers
        leftTargetPower *= maxSpeed;
        rightTargetPower *= maxSpeed;

        //clip to range (-1 -> 1)
        lastLeftPower  = Range.clip(leftTargetPower,  -1, 1);
        lastRightPower = Range.clip(rightTargetPower, -1, 1);


        //force stop when no input is detected
        if (rawTurn == 0 && rawDrive == 0) {
            lastLeftPower = 0;
            lastRightPower = 0;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
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