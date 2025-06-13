package org.firstinspires.ftc.teamcode.mainModules;  //place where the code is located

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MoveRobot {


    //these need to be defined here because they are used in multiple methods
    private DcMotorEx leftFrontDriveEx = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDriveEx = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDriveEx = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDriveEx = null;  //  Used to control the right back drive wheel

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final boolean useVelocity;

    private final boolean protect;

    double wantedAngle = 0;

    double maxSpeed=1;

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
        rightFrontDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_0_CH");
        leftFrontDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_1_CH");
        leftBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_2_CH");
        rightBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_3_CH");

        //set the correct directions for the motors
        leftFrontDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDriveEx.setDirection(DcMotorEx.Direction.REVERSE);


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

    // the main function for moving the robot
    public void move(double heading, double drive, double strafe, double turn,
                     boolean fieldCentric, boolean turnFieldCentric,
                     boolean lockToBackWall, double autoCompensation,
                     boolean speed1, boolean speed2, boolean speed3,
                     double maxRaisedVelocity
    ) {
        if (speed1){
            maxSpeed=0.25;
        }if (speed2){
            maxSpeed=0.5;
        }if (speed3){
            maxSpeed=1;
        }
        maxSpeed = Math.min(maxRaisedVelocity, maxSpeed);



        double x;
        double y;
        double turnCompensation;
        double turnMultiplier = 0.3;
        SlowUpDate turnSlower = new SlowUpDate(20);

        //the robot can constantly compensate for its angle or have it be freely turning
        {
            if (lockToBackWall && turnFieldCentric) {
                //turn to 0°
                turnCompensation = (heading-2* Math.PI) * turnMultiplier;
            } else if (turnFieldCentric) {
                turnCompensation = heading - wantedAngle;
                if (turnSlower.isTurn()) {
                    wantedAngle += turn;
                    telemetry.addData("wanted angle", wantedAngle);
                    telemetry.addData("turn", turn);
                }
            } else {
                wantedAngle = heading; // so if switched to the other the robot wont flick to a distant angle
                turnCompensation = turn;
            }
        }
        // The operator can choose to move the robot relative to the field or to the robot
        if (fieldCentric) {
            // we have had problems with the imu, this prevents the code from crashing during a game if something goes wrong
            if (protect) {
                try {
                    if (lockToBackWall){
                        x = autoCompensation;
                    }
                    else{
                        x = drive * Math.cos(heading) - strafe * Math.sin(heading);
                    }
                    y = drive * Math.sin(heading) + strafe * Math.cos(heading);
                } catch (Exception e) {
                    x = drive;
                    y = strafe;
                }
            } else {
                if (lockToBackWall){
                    x = autoCompensation;
                }
                else{
                    x = drive * Math.cos(heading) - strafe * Math.sin(heading);
                }
                y = drive * Math.sin(heading) + strafe * Math.cos(heading);
            }
        } else {
            x = drive;
            y = strafe;
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
        double maxAngularVelocityRadians = 1972.92;
        double wheelSizeCorrection = 1.2039; // we use 2 different sizes of wheels. We double the wheels on each motor for better grip and there are only 4 of both sizes

        if (useVelocity) {
            // Calculate wheel speeds normalized to the wheels.
            double leftFrontRawSpeed = (leftFrontPowerRaw / max * maxAngularVelocityRadians);
            double leftBackRawSpeed = (leftBackPowerRaw / max * maxAngularVelocityRadians / wheelSizeCorrection);
            double rightFrontRawSpeed = (rightFrontPowerRaw / max * maxAngularVelocityRadians / wheelSizeCorrection);
            double rightBackRawSpeed = (rightBackPowerRaw / max * maxAngularVelocityRadians);   

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
}
class SlowUpDate {
    // System.currentTimeMillis(); return milliseconds long so everything is in long to avoid type conflicts
    private long msBetween = 20;
    private long lastTime = System.currentTimeMillis();

    SlowUpDate(long msBetween){
        this.msBetween = msBetween;
    }

    boolean isTurn(){
        long currentDiff = System.currentTimeMillis() - lastTime;
        if (currentDiff > msBetween) {
            lastTime = System.currentTimeMillis();
            return true;
        } else {
            return false;
        }
    }
}