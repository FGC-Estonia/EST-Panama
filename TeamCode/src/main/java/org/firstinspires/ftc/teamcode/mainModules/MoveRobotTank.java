package org.firstinspires.ftc.teamcode.mainModules;  // The folder where the code is located

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class is a new version of MoveRobot specifically for a "tank drive" robot with only TWO wheels.
 * It assumes you have one motor for the left side and one for the right side.
 * It does not strafe (move side-to-side).
 */
public class MoveRobotTank {

    // These are boxes to hold our two-wheel motors.
    private DcMotorEx leftBackDriveEx = null;   //  The left back drive wheel
    private DcMotorEx rightBackDriveEx = null;  //  The right back drive wheel

    // A box to hold our telemetry (the message writer for the phone screen).
    private final Telemetry telemetry;

    // A box for the robot's maximum speed, starting at 100%.
    double maxSpeed = 1;

    /**
     * These are the "Getting Started" instructions that run when you create a new MoveRobotTank.
     * @param hardwareMap The map of all the robot's connected parts.
     * @param telemetry The tool to send messages to the driver's phone.
     * @param useVelocity A setting to use motor encoders for precise speed (true) or just raw power (false).
     */
    public MoveRobotTank(HardwareMap hardwareMap, Telemetry telemetry, boolean useVelocity){
        // We take the tools we are given and save them.
        this.telemetry = telemetry;
        // Run the instructions to find and set up the motors.
        mapMotors(hardwareMap, useVelocity);
    }

    /**
     * This set of instructions finds the motors on the robot and sets them up correctly.
     * @param hardwareMap The map of all the robot's connected parts.
     * @param useVelocity A setting to use motor encoders or not.
     */
    private void mapMotors(HardwareMap hardwareMap, boolean useVelocity) {
        // We find each motor using the name it was given in the robot's configuration.
        // We are only looking for the back motors now.
        leftBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_2_CH");
        rightBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_3_CH");

        // To make the robot go forward, the motors on one side need to spin the opposite way from the other.
        // We set the left motor to FORWARD and the right motor to REVERSE.
        leftBackDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDriveEx.setDirection(DcMotorEx.Direction.REVERSE);

        // Depending on our setting, we tell the motors to run using precise speed (encoders) or just raw power.
        if (useVelocity) {
            leftBackDriveEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDriveEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            leftBackDriveEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDriveEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * The main instruction for moving the tank-drive robot.
     * @param leftPower The power for the left wheel (from -1.0 to 1.0).
     * @param rightPower The power for the right wheel (from -1.0 to 1.0).
     * @param speed1 Is the slowest speed gear selected?
     * @param speed2 Is the medium speed gear selected?
     * @param speed3 Is the fastest speed gear selected?
     */
    public void move(double leftPower, double rightPower, boolean speed1, boolean speed2, boolean speed3) {
        // This is our gear shifter. It changes the robot's top speed.
        if (speed1){
            maxSpeed = 0.25; // 25% speed
            telemetry.addData("Gear", "Slow (25%)");
        } else if (speed2){
            maxSpeed = 0.5;  // 50% speed
            telemetry.addData("Gear", "Medium (50%)");
        } else if (speed3){
            maxSpeed = 1.0;  // 100% speed
            telemetry.addData("Gear", "Fast (100%)");
        }

        // We calculate the final power by multiplying the joystick power by our speed limit.
        double finalLeftPower = leftPower * maxSpeed;
        double finalRightPower = rightPower * maxSpeed;

        // Send the final power command to the motors!
        leftBackDriveEx.setPower(finalLeftPower);
        rightBackDriveEx.setPower(finalRightPower);

        // Show the power levels on the phone screen.
        telemetry.addData("Left Power", finalLeftPower);
        telemetry.addData("Right Power", finalRightPower);
    }
}
