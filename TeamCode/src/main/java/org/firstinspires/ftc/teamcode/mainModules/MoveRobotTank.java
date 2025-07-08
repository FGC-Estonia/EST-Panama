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
    private double lastTimeCalledDrive = System.nanoTime();
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

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (useVelocity) {
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    private double clamp(double current, double min, double max) {
        return Math.max(min, Math.min(current, max));
    }

    public void drive(double heading, double forward, double turn, boolean speed1, boolean speed2, boolean speed3) {

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


        //final
        double leftPower = forward + turn;
        double rightPower = forward - turn;

        //drive forward using gyro
        if (turn == 0) {
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


        //final clamp to -1 -> 1
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