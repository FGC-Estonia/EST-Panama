package org.firstinspires.ftc.teamcode.mainModules;  //place where the code is located

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.common.util.DriveBaseController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class MoveRobot implements DriveBaseController {


    //these need to be defined here because they are used in multiple methods
    private DcMotorEx leftFrontDriveEx = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDriveEx = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDriveEx = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDriveEx = null;  //  Used to control the right back drive wheel

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final boolean useVelocity;

    private final boolean protect;
    double MAX_ANGULAR_VELOCITY_RADIANS = 1972.92;

        double wantedAngle = 0;

        double maxSpeed=1;

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

        public MoveRobot(boolean protect, HardwareMap hardwareMap, Telemetry telemetry, boolean useVelocity){

            //Pass required objects and a setting to the class
            this.protect = protect;
            this.telemetry = telemetry;
            this.hardwareMap = hardwareMap;
            this.useVelocity = useVelocity;
            mapMotors();
    }

    private void mapMotors() {

        // Mapping motors
        rightFrontDriveEx = hardwareMap.get(DcMotorEx.class, HardwareConstants.RIGHT_FRONT_MOTOR);
        leftFrontDriveEx = hardwareMap.get(DcMotorEx.class, HardwareConstants.LEFT_FRONT_MOTOR);
        leftBackDriveEx = hardwareMap.get(DcMotorEx.class, HardwareConstants.LEFT_BACK_MOTOR);
        rightBackDriveEx = hardwareMap.get(DcMotorEx.class, HardwareConstants.RIGHT_BACK_MOTOR);

        //set the correct directions for the motors
        leftFrontDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDriveEx.setDirection(DcMotorEx.Direction.FORWARD);


        // Depending on settings, the robot will run using velocity or power
        if (useVelocity) {
            leftFrontDriveEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDriveEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDriveEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDriveEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            leftFrontDriveEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDriveEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDriveEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDriveEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    /**
     * Controls the robot's movement based on joystick inputs and current heading.
     * Can operate in field-centric or robot-centric mode.
     *
     * @param heading Current robot heading in radians (e.g., from IMU).
     * @param drive Forward/backward input (-1.0 to 1.0).
     * @param strafe Left/right strafe input (-1.0 to 1.0) - Does tank strafe though?
     * @param turn Turning input (-1.0 to 1.0).
     * @param fieldCentric True if movement should be relative to the field, false for robot-centric.
     * @param driveGear The selected drive gear (LOW, MEDIUM, HIGH) determining max speed.
     */
    public void move(double heading, double drive, double strafe, double turn,
                     boolean fieldCentric,
                     DriveGear driveGear
    ) {
        this.maxSpeed = driveGear.maxSpeed;
        telemetry.addData("Gear", driveGear.telemetryName);

        double x;
        double y;
        double turnCompensation;

        //the robot can constantly compensate for its angle or have it be freely turning
        wantedAngle = heading; // so if switched to the other the robot wont flick to a distant angle
        turnCompensation = turn;

        // The operator can choose to move the robot relative to the field or to the robot
        x = drive;
        y = strafe;

        if (fieldCentric && protect) {
            try {
                x = drive * Math.cos(heading) - strafe * Math.sin(heading);
                y = drive * Math.sin(heading) + strafe * Math.cos(heading);
            } catch (Exception ignored) {
                // Already defaulted to drive/strafe
            }
        } else if (fieldCentric) {
            x = drive * Math.cos(heading) - strafe * Math.sin(heading);
            y = drive * Math.sin(heading) + strafe * Math.cos(heading);
        }

        // Calculates raw power to motors
        double leftFrontPowerRaw = x + y + turnCompensation;
        double leftBackPowerRaw = x - y + turnCompensation;
        double rightFrontPowerRaw = x - y - turnCompensation;
        double rightBackPowerRaw = x + y - turnCompensation;

        // Calculate the maximum absolute power value for normalization
        double maxRawPower = Math.max(
                Math.max(Math.abs(leftFrontPowerRaw), Math.abs(leftBackPowerRaw)),
                Math.max(Math.abs(rightFrontPowerRaw), Math.abs(rightBackPowerRaw))
        );
        // if the power is not over 1, the code will divide by 1, which doesn't affect the end result
        double max = Math.max(maxRawPower, 1);

        if (useVelocity) {
            // Calculate wheel speeds normalized to the wheels.
            double leftFrontRawSpeed = (leftFrontPowerRaw / max * MAX_ANGULAR_VELOCITY_RADIANS);
            double leftBackRawSpeed = (leftBackPowerRaw / max * MAX_ANGULAR_VELOCITY_RADIANS);
            double rightFrontRawSpeed = (rightFrontPowerRaw / max * MAX_ANGULAR_VELOCITY_RADIANS);
            double rightBackRawSpeed = (rightBackPowerRaw / max * MAX_ANGULAR_VELOCITY_RADIANS);

            leftFrontDriveEx.setVelocity(leftFrontRawSpeed * maxSpeed);
            leftBackDriveEx.setVelocity(leftBackRawSpeed * maxSpeed);
            rightFrontDriveEx.setVelocity(rightFrontRawSpeed * maxSpeed);
            rightBackDriveEx.setVelocity(rightBackRawSpeed * maxSpeed);

        } else {
            // Set motor power directly
            leftFrontDriveEx.setPower(leftFrontPowerRaw / max * maxSpeed);
            leftBackDriveEx.setPower(leftBackPowerRaw / max * maxSpeed);
            rightFrontDriveEx.setPower(rightFrontPowerRaw / max * maxSpeed);
            rightBackDriveEx.setPower(rightBackPowerRaw / max * maxSpeed);
        }

    }

    @Override
    public void drive(double imuAngle,
                      double imuPitch,
                      double drive,
                      double strafe,
                      double turn,
                      boolean fieldCentric,
                      DriveGear driveGear) {
        move(imuAngle, drive, strafe, turn, fieldCentric, driveGear);
    }

    public int[] getEncoderPositions() {
        return new int[]{
                rightFrontDriveEx.getCurrentPosition(),
                leftFrontDriveEx.getCurrentPosition(),
                leftBackDriveEx.getCurrentPosition(),
                rightBackDriveEx.getCurrentPosition()
        };
    }

} // Correct closing brace for the `MoveRobot` class.