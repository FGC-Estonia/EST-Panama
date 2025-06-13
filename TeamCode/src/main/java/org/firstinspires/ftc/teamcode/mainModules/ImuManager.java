package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ImuManager {

    //These are used in multiple places so they need to be defined here
    private IMU imu;

    private boolean imuErrorBoolean = false;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final boolean protect;

    public void initImu(){
        if (protect) {
            try {
                // Initializing imu to avoid errors
                imu = hardwareMap.get(IMU.class, "imu");

                RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
                RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

                imu.initialize(new IMU.Parameters(orientationOnRobot));

                imu.resetYaw();
                imuErrorBoolean = false;
            } catch (Exception errorInitIMU) {
                imuErrorBoolean = true;
                telemetry.addData("IMU error", errorInitIMU.getMessage());
            }
        }else {
            imu = hardwareMap.get(IMU.class, "imu");

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu.initialize(new IMU.Parameters(orientationOnRobot));

            imu.resetYaw();
            imuErrorBoolean = false;
        }
    }

    public ImuManager(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        initImu();
    }

    public void resetImu(){
        if (protect) {
            try {
                imu.resetYaw();
            } catch (Exception resetException) {
                telemetry.addLine(resetException.getMessage());
            }
        }
        else {
            imu.resetYaw();
        }
    }

    public double getYawRadians(){
        double lastAngle = 0; //if the imu fails in the middle of the game, it will not flick to an angle because the imuManager returned 0, instead it will just stop working safely

        //if the imu has failed it will attempt to restart it.
        if (imuErrorBoolean) {
            initImu();
        }

        if (protect) {
            try {
                lastAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                return lastAngle;
            } catch (Exception errorIMU) {
                telemetry.addData("IMU ERROR", errorIMU.getMessage());
                return lastAngle;
            }
        }else {
            lastAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            return lastAngle;
        }
    }
}
