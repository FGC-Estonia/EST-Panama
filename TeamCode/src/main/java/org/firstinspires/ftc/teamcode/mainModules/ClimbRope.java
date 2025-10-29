package org.firstinspires.ftc.teamcode.mainModules;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class ClimbRope {
    private DcMotorEx climbMotor = null;
    private Servo leftMoveServo;
    private Servo rightMoveServo;
    private TouchSensor magneticLimitSwitch;
    private final boolean protect;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final double CLIMB_DOWN_RATE = 0.2;
    private final double STAY_ON_RATE = 0.1;
    boolean resettingPosition = false;
    private int storedHomePositionTicks = 0;
    // Define constants for your climbMotor and gearing
    private static final double REV_HD_HEX_MOTOR_TICKS_PER_MOTOR_REV = 28.0; // From REV HD Hex Motor spec
    private static final double GEAR_RATIO_1 = 5.23;
    private static final double GEAR_RATIO_2 = 5.23;
    private static final double GEAR_RATIO_3 = 3.61;

    // Calculate the total ticks per revolution for the final output shaft
    private static final double TOTAL_GEAR_RATIO = GEAR_RATIO_1 * GEAR_RATIO_2 * GEAR_RATIO_3;
    private static final int TICKS_PER_OUTPUT_SHAFT_REVOLUTION = (int) (REV_HD_HEX_MOTOR_TICKS_PER_MOTOR_REV * TOTAL_GEAR_RATIO);

    public ClimbRope(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotors();
    }

    private void mapMotors() {
        try {
            leftMoveServo = hardwareMap.get(Servo.class, "Servo_Port_5_CH");
            leftMoveServo.setDirection(Servo.Direction.REVERSE);
        } catch (Exception e) {
            leftMoveServo = null;
        }

        try {
            rightMoveServo = hardwareMap.get(Servo.class, "Servo_Port_4_CH");
            rightMoveServo.setDirection(Servo.Direction.FORWARD);
        } catch (Exception e) {
            rightMoveServo = null;
        }

        try {
            magneticLimitSwitch = hardwareMap.get(TouchSensor.class, "Digital_Port_1_CH");
        } catch (Exception e) {
            magneticLimitSwitch = null;
        }


        climbMotor = hardwareMap.get(DcMotorEx.class, HardwareConstants.ROPE_MOTOR);

        climbMotor.setDirection(DcMotor.Direction.REVERSE);

        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    // ROTATING TO POS DOESN'T REALLY WORK
    public void rotateToHome() {
        int currentFullPosition = climbMotor.getCurrentPosition();
        int currentAngularOffset = currentFullPosition % TICKS_PER_OUTPUT_SHAFT_REVOLUTION;

        // Handle negative currentAngularOffset
        if (currentAngularOffset < 0) {
            currentAngularOffset += TICKS_PER_OUTPUT_SHAFT_REVOLUTION;
        }

        // Calculate the difference needed to reach the redDotHomeAngularOffset
        int deltaAngle = storedHomePositionTicks - currentAngularOffset;

        // Adjust deltaAngle for the shortest path around the circle
        // Example: If current is 270 degrees and target is 10 degrees, moving +100 degrees
        // is shorter than -260 degrees.
        if (deltaAngle > TICKS_PER_OUTPUT_SHAFT_REVOLUTION / 2) {
            deltaAngle -= TICKS_PER_OUTPUT_SHAFT_REVOLUTION;
        } else if (deltaAngle < -TICKS_PER_OUTPUT_SHAFT_REVOLUTION / 2) {
            deltaAngle += TICKS_PER_OUTPUT_SHAFT_REVOLUTION;
        }

        // The new target is the current full position plus this small angular adjustment
        int targetFullPosition = currentFullPosition + deltaAngle;

        rotateToPosition(targetFullPosition);
    }
    public void rotateToPosition(int targetTicks) {
        // This tells the climbMotor controller to use the encoders to reach a specific target.
        climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // The motors will try to reach this absolute encoder count.
        climbMotor.setTargetPosition(targetTicks);

        // This power (from 0.0 to 1.0) acts as the maximum speed the motors will use
        // to try and reach their target. The controller will vary the power to slow down
        // as it approaches the target.
        climbMotor.setPower(1.0); // Full power for movement

        // Wait until the motors reach their target or a timeout occurs.
        long startTime = System.currentTimeMillis();
        long timeoutMillis = 10000; // 10 seconds timeout. Adjust this value based on your mechanism's speed and travel distance.

        // Loop while the OpMode is active, at least one motor is still moving, AND the timeout hasn't been reached.
        while ((climbMotor.isBusy()) && (System.currentTimeMillis() - startTime < timeoutMillis)) {
            // Provide real-time feedback on the Driver Station for debugging.
            telemetry.addData("Left Target", climbMotor.getTargetPosition());
            telemetry.update();
        }

        // Stop the motors once they reach the target or the loop exits (e.g., due to timeout).
        climbMotor.setPower(0);

        // Set motors back to RUN_WITHOUT_ENCODER mode.
        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public int rememberHomePosition(){
        storedHomePositionTicks = climbMotor.getCurrentPosition() % TICKS_PER_OUTPUT_SHAFT_REVOLUTION;
        if (storedHomePositionTicks < 0) {
            storedHomePositionTicks += TICKS_PER_OUTPUT_SHAFT_REVOLUTION;
        }
    return storedHomePositionTicks;
    }

    public void ropeClimbing(int direction, float stick) {
        double thingy = 0.6;
        if (resettingPosition) {return;}

        double power;

        if (direction == 2) {  // Climb up
            power = -1.0;
        } else if (direction == -1) {  // Climb down slowly
            power = CLIMB_DOWN_RATE;
        } else if (direction == 1) {
            power = -STAY_ON_RATE;
        } else if (direction == 3) {
            power = -(Math.pow(stick, 1));
        } else {
            power = 0;
        }

        climbMotor.setPower(power);
    }

    public void slide(boolean open) {
        if (leftMoveServo == null || rightMoveServo == null) {
            telemetry.addData("Slide", "Servos not mapped - abort");
            telemetry.update();
            try {
                Thread.sleep(600);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
            }
            return;
        }



        if (open) {
            telemetry.addData("Slide", "Opening...");
            telemetry.addData("magnet isPressed:", magneticLimitSwitch.isPressed());
            telemetry.update();

            // extend
            leftMoveServo.setPosition(1.0);
            rightMoveServo.setPosition(1.0);

            // Wait until magnetic limit switch detects magnet (active-low => getState()==false)
            final long timeoutMs = 5000; // 3 seconds max
            long start = System.currentTimeMillis();
            boolean detected = false;

            if (magneticLimitSwitch != null) {
                while (System.currentTimeMillis() - start < timeoutMs) {
                    // active-low sensor: false means magnet present / limit reached
                    try {
                        if (magneticLimitSwitch.isPressed()) {
                            detected = true;
                            break;
                        }
                    } catch (Exception e) {
                        // sensor read failed; bail to timeout
                        break;
                    }
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                }
            } else {
                // No limit switch: wait a conservative default time
                try {
                    Thread.sleep(600);
                    detected = true; // assume done after time
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                }
            }

            telemetry.addData("Slide", detected ? "Limit reached" : "Timeout");
            telemetry.update();

            // set to hold position
            leftMoveServo.setPosition(0.5);
            rightMoveServo.setPosition(0.5);
        } else {
            telemetry.addData("Slide", "Closing...");
            telemetry.update();

            // retract
            leftMoveServo.setPosition(0.0);
            rightMoveServo.setPosition(0.0);

            // wait fixed 2000 ms for retract travel
            try {
                Thread.sleep(4000);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
            }

            // set to hold position
            leftMoveServo.setPosition(0.5);
            rightMoveServo.setPosition(0.5);

            telemetry.addData("Slide", "Closed - holding");
            telemetry.update();
        }
    }

    public void showDigPort() {
        if (magneticLimitSwitch == null) {telemetry.addData("not mapped magnet", ""); return;}
        telemetry.addData("magnet sensor:", magneticLimitSwitch.getValue());
        telemetry.addData("magnet isPressed:", magneticLimitSwitch.isPressed());
    }

}