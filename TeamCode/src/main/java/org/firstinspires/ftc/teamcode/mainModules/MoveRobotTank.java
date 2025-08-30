package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.DriveBaseController;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;
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
    private final boolean protect; // Will keep this as it was part of original constructor

    // --- Drive Constants ---
    private static final double MAX_TURN_SPEED = 0.5; // Kept, might be used by other logic if restored
    private static final double MAX_TURN_DURING_CURVE = 0.2; // Kept
    private static final double DRIVE_DEADZONE = 0.05; // Joystick deadband
    private static final double TURN_DEADZONE = 0.02; // Joystick deadband
    private static final double STATIONARY_TURN_THRESHOLD = 0.05; // Kept
    private static final double CURVATURE_DRIVE_FACTOR = 0.8; // Kept
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
        public final double turnSpeed; // This 'turnSpeed' from enum seems to be a direct speed cap for turning
        public final String telemetryName;

        DriveGear(double maxSpeed, double turnSpeed, String telemetryName) {
            this.maxSpeed = maxSpeed;
            this.turnSpeed = turnSpeed;
            this.telemetryName = telemetryName;
        }
    }
    public MoveRobotTank(boolean protect, HardwareMap hardwareMap, Telemetry telemetry, boolean useVelocity) {
        this.protect = protect; // Kept as per original
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.useVelocity = useVelocity;
        mapMotors();
    }

    private void mapMotors() {
        leftDrive = hardwareMap.get(DcMotorEx.class, HardwareConstants.TANK_LEFT_MOTOR);
        rightDrive = hardwareMap.get(DcMotorEx.class, HardwareConstants.TANK_RIGHT_MOTOR);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

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
                      double frontBack,      // Raw joystick input for forward/backward
                      double leftRight,      // Raw joystick for strafe (ignored by tank)
                      double turnInput,      // Raw joystick input for turning
                      boolean fieldCentric,  // Usually for Mecanum/Omni, ignored by basic tank
                      DriveGear gear) {

        // Tank drive ignores strafe (leftRight) and fieldCentric for basic operation
        this.maxSpeed = gear.maxSpeed;
        // The original code had: this.turnSpeed = (gear.telemetryName.equals("1") ? 0.4 : gear.telemetryName.equals("2") ? 0.6 : 0.8);
        // This seems to override the gear.turnSpeed. Using gear.turnSpeed directly for clarity if that was the intent.
        // If the string comparison was intended, it should be based on gear.telemetryName, not "1", "2".
        // For simplicity, let's assume gear.turnSpeed from the enum is the intended factor for turnInput.
        this.turnSpeed = gear.turnSpeed; // Using the turnSpeed from the DriveGear enum
        telemetry.addData("Gear", gear.telemetryName + " (MaxSpeed: " + maxSpeed + ", TurnSpeed: " + turnSpeed + ")");


        // =================================================================================
        // SIMPLIFIED DRIVE LOGIC STARTS HERE
        // =================================================================================

        // 1. Apply deadzone to raw joystick inputs
        //    Assuming negative frontBack is forward, positive turnInput is right turn. Adjust if needed.
        double processedDriveInput = applyDeadzone(-frontBack, DRIVE_DEADZONE);
        double processedTurnInput = applyDeadzone(turnInput, TURN_DEADZONE);

        // 2. Calculate basic motor powers
        //    Directly use processed inputs. 'turnSpeed' from gear acts as a scaling factor for turn.
        double leftTargetPower = processedDriveInput + (processedTurnInput * this.turnSpeed);
        double rightTargetPower = processedDriveInput - (processedTurnInput * this.turnSpeed);

        // 3. Normalize powers if they exceed magnitude of 1.0 (maintains turn ratio)
        double maxMagnitude = Math.max(Math.abs(leftTargetPower), Math.abs(rightTargetPower));
        if (maxMagnitude > 1.0) {
            leftTargetPower /= maxMagnitude;
            rightTargetPower /= maxMagnitude;
        }

        // 4. Apply overall maxSpeed scaling from the current gear
        leftTargetPower *= this.maxSpeed;
        rightTargetPower *= this.maxSpeed;

        // At this point, leftTargetPower and rightTargetPower are the simplified outputs.
        // The more complex logic below (slew rate, heading hold, auto-brake) is kept
        // but will now operate on these simplified target powers if those features are enabled/triggered.

        // =================================================================================
        // END OF SIMPLIFIED DRIVE LOGIC - Original advanced features follow,
        // but their *inputs* (drive, turn) are now simplified.
        // =================================================================================


        // --- Original code's further processing (kept as requested, but inputs to them are now simplified) ---

        // The following lines from the original code that further modify `drive` and `turn`
        // before they become `leftTargetPower` and `rightTargetPower` are effectively bypassed
        // by the simplified block above.
        // If you want to use slew rates, cubic scaling etc., you'd integrate them into the
        // "SIMPLIFIED DRIVE LOGIC" block or revert that block.

        /*
        // before limiting:
        double rawDrive = -applyDeadzone(frontBack, DRIVE_DEADZONE);
        double rawTurn  = applyDeadzone(turnInput, TURN_DEADZONE) * turnSpeed; // turnSpeed here is from this.turnSpeed

        // --- Counterstrife logic for drive and turn --- (Kept commented as in original)
        //boolean reversingDrive = (Math.signum(rawDrive) != Math.signum(lastDrive)) && lastDrive != 0 && (Math.abs(rawDrive) > 0.05);
        //boolean reversingTurn  = (Math.signum(rawTurn)  != Math.signum(lastTurn)) && lastTurn != 0 && (Math.abs(rawTurn) > 0.05);

        // cubic scaling: (Original processing)
        double drive = -driverLimiter.calculate(Math.pow(rawDrive, 3)); // SLEW RATE + CUBIC
        //boolean stationaryTurn = Math.abs(drive) < STATIONARY_TURN_THRESHOLD;

        double rawTurnCubed = Math.pow(rawTurn, 3); // CUBIC
        //double turn = stationaryTurn ? rawTurnCubed : rawTurnCubed; // STATIONARY TURN LOGIC
        double turn = rawTurnCubed; // Assuming turnLimiter not applied here in original, or was intended for gyro?

        drive = (rawDrive == 0) ? 0 : drive;
        turn  = (rawTurn == 0) ? 0 : turn;

        lastDrive = drive; // Part of slew/state logic
        lastTurn  = turn;  // Part of slew/state logic

        telemetry.addData("raw drive (original path)", frontBack);
        telemetry.addData("deadzone drive (original path)", rawDrive);
        telemetry.addData("final drive input (original path with slew/cubic)", drive);

        // Original calculation of leftTargetPower, rightTargetPower based on potentially slew-rated/cubed 'drive' and 'turn'
        // leftTargetPower  = drive + turn;
        // rightTargetPower = drive - turn;
        */


        // --- The simplified logic above has already calculated leftTargetPower and rightTargetPower ---
        // --- We will now use these values for the subsequent features like heading hold and auto-brake ---

        // Store these potentially final values before auto-brake or heading hold modifies them further.
        // These are the values that would go to the motors if no heading hold or auto-brake was active.
        double desiredLeftPower = leftTargetPower;
        double desiredRightPower = rightTargetPower;


        telemetry.addData("Simplified Left Power (pre-features)", "%.2f", desiredLeftPower);
        telemetry.addData("Simplified Right Power (pre-features)", "%.2f", desiredRightPower);


        /* Original Curvature Drive Logic - Kept commented as it was in original
        if (stationaryTurn) { // 'stationaryTurn' would need to be calculated based on 'processedDriveInput' if used
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
        */


        // --- Heading hold ---
        // This will now use the 'desiredLeftPower' and 'desiredRightPower' from the simplified path
        // as the basis for determining if heading hold should engage.
        double avgInput = (desiredLeftPower + desiredRightPower) / 2.0 / this.maxSpeed; // Approx. average drive input before gear scaling
        double diff = Math.abs(desiredLeftPower - desiredRightPower) / this.maxSpeed; // Approx. turn input before gear scaling

        if (diff < 0.05 && Math.abs(avgInput) > 0.05) { // Thresholds might need tuning
            if (!headingHoldEnabled) {
                wantedHeading = imuAngle;
            }
            headingHoldEnabled = true;
        } else {
            headingHoldEnabled = false;
        }

        if (headingHoldEnabled) {
            double headingError = normalizeRadians(wantedHeading - imuAngle);
            double rawCorrection = headingError * HEADING_KP;
            double correction = gyroLimiter.calculate(rawCorrection); // Slew rate for gyro correction
            desiredLeftPower  -= correction * this.maxSpeed; // Apply correction considering gear speed
            desiredRightPower += correction * this.maxSpeed;
            telemetry.addData("Heading Hold", "ACTIVE (Error: %.2f, Corr: %.2f)", headingError, correction);
        } else {
            telemetry.addData("Heading Hold", "INACTIVE");
        }
        // Powers are now potentially modified by heading hold


        // --- Final power clipping before applying to motors ---
        // This was 'lastLeftPower' and 'lastRightPower' in the original.
        // Renaming for clarity in this modified flow.
        double finalLeftMotorOutput = Range.clip(desiredLeftPower, -1, 1);
        double finalRightMotorOutput = Range.clip(desiredRightPower, -1, 1);


        // --- Auto-brake when sticks are released but wheels still spinning ---
        // This logic checks the raw joystick inputs (frontBack, turnInput)
        // If they are zero, it might try to brake.
        // It's important that `applyDeadzone(-frontBack, DRIVE_DEADZONE)` and `applyDeadzone(turnInput, TURN_DEADZONE)`
        // both result in 0 for this condition to be fully effective for "sticks released".
        if (applyDeadzone(-frontBack, DRIVE_DEADZONE) == 0 && applyDeadzone(turnInput, TURN_DEADZONE) == 0) {
            double leftVel  = leftDrive.getVelocity();
            double rightVel = rightDrive.getVelocity();

            double stopThreshold = 80.0; // TPS
            if (Math.abs(leftVel) > stopThreshold || Math.abs(rightVel) > stopThreshold) {
                // If moving, apply gentle braking power
                double brakePowerLeft  = -Math.signum(leftVel)  * 0.05; // Reduced brake power for gentler stop
                double brakePowerRight = -Math.signum(rightVel) * 0.05;

                if (useVelocity) {
                    // For velocity control, setting a very low opposing velocity or zero might be better
                    // Or switch to power control for braking if more effective.
                    // This part might need tuning based on motor behavior.
                    leftDrive.setVelocity(brakePowerLeft * MAX_MOTOR_VELOCITY_TPS * 0.1); // Small opposing velocity
                    rightDrive.setVelocity(brakePowerRight * MAX_MOTOR_VELOCITY_TPS * 0.1);
                } else {
                    leftDrive.setPower(brakePowerLeft);
                    rightDrive.setPower(brakePowerRight);
                }
                telemetry.addData("Auto-Brake", "ACTIVE (L:%.2f, R:%.2f)", brakePowerLeft, brakePowerRight);
                // Storing these as last power might be relevant if other logic depends on it
                lastLeftPower = brakePowerLeft;
                lastRightPower = brakePowerRight;
                return; // Exit after applying brake power
            } else {
                // If not moving significantly, just ensure motors are set to zero
                finalLeftMotorOutput = 0;
                finalRightMotorOutput = 0;
                telemetry.addData("Auto-Brake", "STICKS ZERO, MOTORS NEAR ZERO");
            }
        } else {
            telemetry.addData("Auto-Brake", "INACTIVE (Driving)");
        }

        // Store the final outputs for potential use by other systems or for `lastLeft/RightPower`
        lastLeftPower = finalLeftMotorOutput;
        lastRightPower = finalRightMotorOutput;


        // --- Set motor powers/velocities ---
        if (useVelocity) {
            leftDrive.setVelocity(lastLeftPower * MAX_MOTOR_VELOCITY_TPS);
            rightDrive.setVelocity(lastRightPower * MAX_MOTOR_VELOCITY_TPS);
        } else {
            leftDrive.setPower(lastLeftPower);
            rightDrive.setPower(lastRightPower);
        }

        telemetry.addData("Final Left Output", "%.2f", lastLeftPower);
        telemetry.addData("Final Right Output", "%.2f", lastRightPower);
        if (useVelocity) {
            telemetry.addData("Left Actual Velocity", "%.2f TPS", leftDrive.getVelocity());
            telemetry.addData("Right Actual Velocity", "%.2f TPS", rightDrive.getVelocity());
        }
        // telemetry.addData("Heading Error", normalizeRadians(wantedHeading - imuAngle)); // Already part of heading hold telemetry
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