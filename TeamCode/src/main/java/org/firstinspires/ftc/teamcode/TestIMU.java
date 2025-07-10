package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Test IMU")
public class TestIMU extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Get the IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Set up the orientation of your control hub
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);

        telemetry.addLine("IMU initialized.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get angles
            double yaw   = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
            double roll  = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

            // Display on telemetry
            telemetry.addData("Yaw", "%.2f", yaw);
            telemetry.addData("Pitch", "%.2f", pitch);
            telemetry.addData("Roll", "%.2f", roll);
            telemetry.update();
        }
    }
}