package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.util.ImuManager;
import org.firstinspires.ftc.teamcode.common.util.Presses;
import org.firstinspires.ftc.teamcode.mainModules.*;
import org.firstinspires.ftc.teamcode.common.util.DriveBaseController;
import static org.firstinspires.ftc.teamcode.mainModules.MoveRobot.DriveGear;

@TeleOp(name = "Main Code Estonia Panama")
public class EstoniaPanama extends LinearOpMode {

    // --- Runtime state ---
    private int climbingDirection = 0;
    private int collectingDirection = 0;
    private boolean fieldCentric = false;
    private boolean isFlagRaised = false;
    private boolean isSpinningWheel = false;
    private boolean debugTelemetry = false;
    private int gear = 1;

    private final ElapsedTime runtime = new ElapsedTime();
    private DriveBaseController driveBase;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean protect = true;

        // --- Core systems ---
        driveBase = new MoveRobot(protect, hardwareMap, telemetry, true);
        ImuManager imuManager = new ImuManager(protect, hardwareMap, telemetry, true);
        BallPusher ballPusher = new BallPusher(hardwareMap, telemetry);
        SpinWheel spinWheel = new SpinWheel(hardwareMap, telemetry);

        // --- Optional subsystems ---
        ClimbRope climbRope = null;
        RaiseFlag raiseFlag = null;
        CollectBalls collectBalls = null;

        boolean ropeClimbingAttached = false;
        boolean raiseFlagAttached = false;
        boolean collectBallsAttached = false;

        try {
            climbRope = new ClimbRope(protect, hardwareMap, telemetry);
            ropeClimbingAttached = true;
        } catch (Exception ignored) {
            telemetry.log().add("Rope climbing not attached");
        }

        try {
            raiseFlag = new RaiseFlag(hardwareMap, telemetry);
            raiseFlagAttached = true;
        } catch (Exception ignored) {
            telemetry.log().add("Flag raise not attached");
        }

        try {
            collectBalls = new CollectBalls(protect, hardwareMap, telemetry);
            collectBallsAttached = true;
        } catch (Exception ignored) {
            telemetry.log().add("Collector not attached");
        }

        // --- Controls (Presses wrappers) ---
        Presses g1Share = new Presses();         // field-centric toggle
        Presses g1Options = new Presses();       // IMU reset
        Presses g1LeftBumper = new Presses();    // gear down
        Presses g1RightBumper = new Presses();   // gear up

        Presses g2Square = new Presses();        // spin wheel
        Presses g2Triangle = new Presses();      // rope hold
        Presses g2DpadLeft = new Presses();      // climb home
        Presses g2DpadRight = new Presses();     // remember home
        Presses g2Share = new Presses();         // flag raise

        telemetry.update();
        waitForStart();

        // --- Start timing after match start ---
        runtime.reset();

        while (opModeIsActive()) {

            // === Combo debug toggle ===
            debugTelemetry = Presses.comboToggle(debugTelemetry, gamepad1.options, gamepad1.share);
            if (Presses.comboPressed(gamepad1.options, gamepad1.share))
                gamepad1.rumble(0.4, 0.4, 100);

            // === IMU reset (single press) ===
            if (g1Options.pressed(gamepad1.options)) {
                imuManager.resetImu();
                gamepad1.rumble(0.6, 0.6, 200);
            }

            // === Drive axes ===
            double imuAngle = imuManager.getYawRadians();
            double imuPitch = imuManager.getPitchRadians();
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            // === Field-centric toggle ===
            if (g1Share.pressed(gamepad1.share)) {
                fieldCentric = !fieldCentric;
                if (fieldCentric) gamepad1.rumble(1.0, 1.0, 400);
                else gamepad1.rumble(0.6, 0.6, 100);
            }

            // === Rope climbing ===
            boolean holdingOnRope = g2Triangle.toggle(gamepad2.triangle);

            if (holdingOnRope) climbingDirection = 1;
            else if (gamepad2.left_bumper) climbingDirection = 2;
            else if (gamepad2.right_bumper) climbingDirection = -1;
            else if (Math.abs(gamepad2.left_stick_y) > 0.05) climbingDirection = 3;
            else climbingDirection = 0;

            if (ropeClimbingAttached)
                climbRope.ropeClimbing(climbingDirection, -gamepad2.left_stick_y);

            if (g2DpadRight.pressed(gamepad2.dpad_right) && ropeClimbingAttached)
                climbRope.rememberHomePosition();
            if (g2DpadLeft.pressed(gamepad2.dpad_left) && ropeClimbingAttached)
                climbRope.rotateToHome();

            // === Spin wheel ===
            if (g2Square.toggle(gamepad2.square)) {
                spinWheel.spin(true);
                isSpinningWheel = true;
            } else {
                spinWheel.stop();
                isSpinningWheel = false;
            }

            // === Collect balls ===
            if (collectBallsAttached) {
                if (gamepad2.right_trigger > 0) collectingDirection = 1;
                else if (gamepad2.left_trigger > 0) collectingDirection = -1;
                else collectingDirection = 0;
                collectBalls.collectingBalls(collectingDirection);
            }

            // === Ball pusher ===
            double stickY = gamepad2.right_stick_y;
            if (stickY > 0.3) ballPusher.setMotorStatuses(1);
            else if (stickY < -0.3) ballPusher.setMotorStatuses(0);
            else ballPusher.setMotorStatuses(0.5);

            // === Flag raising ===
            boolean needFlagRaised = g2Share.toggle(gamepad2.share);
            if (raiseFlagAttached) {
                if (needFlagRaised && !isFlagRaised) {
                    raiseFlag.setPos(1);
                    isFlagRaised = true;
                } else if (!needFlagRaised && isFlagRaised) {
                    raiseFlag.setPos(0);
                    isFlagRaised = false;
                }
            }

            // === Gear switching ===
            if (g1LeftBumper.pressed(gamepad1.left_bumper) && gear > 1) gear--;
            if (g1RightBumper.pressed(gamepad1.right_bumper) && gear < 3) gear++;

            DriveGear currentDriveGear =
                    (gear == 1) ? DriveGear.LOW :
                            (gear == 2) ? DriveGear.MEDIUM : DriveGear.HIGH;

            // === Drive control ===
            driveBase.drive(imuAngle, imuPitch, strafe, drive, turn, fieldCentric, currentDriveGear);

            // === Telemetry ===
            telemetry.addLine("— Main Telemetry —");
            telemetry.addData("Gear", gear);
            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("Heading", "%.1f°", imuAngle * 180 / Math.PI);

            if (debugTelemetry) {
                telemetry.addLine("— Debug Telemetry —");
                telemetry.addData("Pitch", "%.1f°", imuPitch * 180 / Math.PI);
                telemetry.addData("Drive Input", "%.2f", drive);
                telemetry.addData("Strafe Input", "%.2f", strafe);
                telemetry.addData("Turn Input", "%.2f", turn);
                telemetry.addData("CollectDir", collectingDirection);
                telemetry.addData("ClimbDir", climbingDirection);
                telemetry.addData("Flag", isFlagRaised);
                telemetry.addData("Wheel", isSpinningWheel);
            }

            telemetry.update();
        }
    }
}