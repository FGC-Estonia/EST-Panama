package org.firstinspires.ftc.teamcode;  //place where the code is located

/* Damn,

There is a huge bug in our code:
    ,__                   __
    '~~****Nm_    _mZ*****~~
            _8@mm@K_
           W~@`  '@~W
          ][][    ][][
    gz    'W'W.  ,W`W`    es
  ,Wf    gZ****MA****Ns    VW.
 gA`   ,Wf     ][     VW.   'Ms
Wf    ,@`      ][      '@.    VW
M.    W`  _mm_ ][ _mm_  'W    ,A
'W   ][  i@@@@i][i@@@@i  ][   W`
 !b  @   !@@@@!][!@@@@!   @  d!
  VWmP    ~**~ ][ ~**~    YmWf
    ][         ][         ][
  ,mW[         ][         ]Wm.
 ,A` @  ,gms.  ][  ,gms.  @ 'M.
 W`  Yi W@@@W  ][  W@@@W iP  'W
d!   'W M@@@A  ][  M@@@A W`   !b
@.    !b'V*f`  ][  'V*f`d!    ,@
'Ms    VW.     ][     ,Wf    gA`
  VW.   'Ms.   ][   ,gA`   ,Wf
   'Ms    'V*mmWWmm*f`    gA`
*/

/* ======================
   Imports (external modules & utilities)
   ====================== */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.util.ImuManager;
import org.firstinspires.ftc.teamcode.mainModules.MoveRobot;
import org.firstinspires.ftc.teamcode.common.util.Presses;
import org.firstinspires.ftc.teamcode.mainModules.ClimbRope;
import org.firstinspires.ftc.teamcode.mainModules.CollectBalls;
import org.firstinspires.ftc.teamcode.mainModules.PoseEstimator;
import org.firstinspires.ftc.teamcode.mainModules.BallPusher;
import org.firstinspires.ftc.teamcode.mainModules.SpinWheel;
import org.firstinspires.ftc.teamcode.mainModules.RaiseFlag;
import org.firstinspires.ftc.teamcode.mainModules.Autonomous;

import org.firstinspires.ftc.teamcode.common.util.DriveBaseController;
import static org.firstinspires.ftc.teamcode.mainModules.MoveRobotTank.DriveGear;

/* ======================
   Opmode annotation + class declaration
   ====================== */
@TeleOp(name = "Main code Estonia Panama")
// allows to display the code in the driver station, comment out to remove
public class EstoniaPanama extends LinearOpMode { //file name is EstoniaPanamas.java    extends the prebuilt LinearOpMode by rev to run
    /* ======================
       Fields / State
       ====================== */
    int climbingDirection = 0; // 0 - stop, 1 - stay on rope, 2 - up, -1 - down, 3 - joystick
    int collectingDirection = 0; // 0 - stop, 1 - in, -1 - out

    int[] lastDriveMotorPositions = {0, 0, 0, 0};

    boolean isSpinningWheel = false;
    ElapsedTime runtime = new ElapsedTime();
    float spinWheelStartTime = (float) runtime.seconds();
    int gear = 1;
    // Robot geometry / encoder constants
    private static final double TICKS_PER_REV = 560.0; // TICKS_PER_REV: encoder ticks per motor revolution
    private static final double WHEEL_DIAMETER = 0.09; // meters, replace with your wheel diameter
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // robot geometry for kinematics: half distances (meters) - replace with your robot measurements
    private static final double HALF_WHEELBASE = 0.155;  // half distance front-back (m) - replace
    private static final double HALF_TRACK = 0.2; // half distance left-right (m) - replace

    boolean fieldCentric = false;
    DriveBaseController driveBase;
    boolean isFlagRaised = false;

    /* ======================
       Main opmode loop
       ====================== */
    @Override
    public void runOpMode() throws InterruptedException {
        boolean protect = true;

        // --- Drive base init ---
        driveBase = new MoveRobot(protect, hardwareMap, telemetry, true);

        // --- subsystem placeholders (may remain null if hardware absent) ---
        ClimbRope climbRope = null;
        boolean ropeClimbingAttached = false; // leave at false, it detects automatically
        RaiseFlag raiseFlag = null;
        boolean raiseFlagAttached = false;  // leave at false, it detects automatically
        CollectBalls collectBalls = null;
        boolean collectBallsAttached = false; // leave at false, it detects automatically


        // --- Core managers & modules initialization ---
        ImuManager imuManager = new ImuManager(protect, hardwareMap, telemetry, true);
        PoseEstimator poseEstimator = new PoseEstimator(imuManager, HALF_WHEELBASE, HALF_TRACK);
        BallPusher ballPusher = new BallPusher(hardwareMap, telemetry);
        SpinWheel spinWheel = new SpinWheel(hardwareMap, telemetry);
        Autonomous autonomous = new Autonomous(driveBase, hardwareMap,telemetry);

        // --- Try to attach optional modules (safe to fail) ---
        try {
            climbRope = new ClimbRope(protect, hardwareMap, telemetry);
            ropeClimbingAttached = true;
        } catch (Exception e) {
            telemetry.log().add("ClimbRope hardware not found — rope climb disabled");
        }

        try {
            raiseFlag = new RaiseFlag(hardwareMap, telemetry);
            raiseFlagAttached = true;
        } catch (Exception e) {
            telemetry.log().add("ClimbRope hardware not found — rope climb disabled");
        }

        try {
            collectBalls = new CollectBalls(protect, hardwareMap, telemetry);
            collectBallsAttached = true;
        } catch (Exception e) {
            telemetry.log().add("Collecting balls hardware not found — collecting balls disabled");
        }

         /* ======================
           Controls: Presses wrappers and toggles
           (grouped and annotated for readability — no logic changed)
           ====================== */

        // Unused controls reserved for future use
        Presses gamepad1_left_trigger = new Presses();
        Presses gamepad1_right_trigger = new Presses();
        Presses gamepad2_cross = new Presses();
        Presses gamepad1_dpad_left = new Presses();
        Presses gamepad1_dpad_right = new Presses();
        Presses gamepad1_dpad_up = new Presses();
        Presses gamepad1_dpad_down = new Presses();
        Presses gamepad2_dpad_up = new Presses();
        Presses gamepad2_dpad_down = new Presses();
        Presses gamepad2_left_bumper = new Presses();
        // Drive speed toggle group (replaced by drivegear)
        Presses.ToggleGroup speedSelectToggle = new Presses.ToggleGroup();
        Presses gamepad1_square = new Presses(speedSelectToggle);
        Presses gamepad1_triangle = new Presses(speedSelectToggle);
        Presses gamepad1_circle = new Presses(speedSelectToggle);
        Presses gamepad1_cross = new Presses(speedSelectToggle);
        gamepad1_triangle.setToggleTrue();//set default value

        // Controls for rope climbing
        Presses gamepad2_triangle = new Presses();
        // Also using:
        // > gamepad2.left_bumper   - climb up
        // > gamepad2.right_bumper  - climb down
        // > gamepad2.left_stick_y  - manual joystick control (when abs > 0.05)
        // Resetting ropeclimb position:
        Presses gamepad2_dpad_left = new Presses();
        Presses gamepad2_dpad_right = new Presses();

        // Controls for spinning wheel
        Presses gamepad2_square = new Presses();

        // Controls for ball pusher:
        // > gamepad2.right_stick_y   - joystick control

        // Controls for collecting balls:
        // > gamepad2.right_trigger   - suck balls in
        // > gamepad2.left_trigger    - let balls out

        // Controls for drive gear
        Presses gamepad1_right_bumper = new Presses();
        Presses gamepad1_left_bumper = new Presses();

        // Controls for flag raising
        Presses gamepad2_share = new Presses();

        // Controls for fieldcentric toggle and gyro reset
        Presses gamepad1_share = new Presses();
        Presses gamepad1_options = new Presses();


        telemetry.update();
        waitForStart(); //everything has been initialized, waiting for the start button
        while (opModeIsActive()) { // main loop

            //gyro reset
            if (gamepad1.options) {
                imuManager.resetImu();
            }

            //move robot
            double imuAngle = imuManager.getYawRadians();
            double imuPitch = imuManager.getPitchRadians();
            double drive = -gamepad1.left_stick_x;
            double strafe = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            // FieldCentric rumble
            if (gamepad1_share.pressed(gamepad1.share)) {
                fieldCentric = !fieldCentric;
                if (fieldCentric) {
                    // One long 500 ms rumble when turning ON
                    gamepad1.rumble(1.0, 1.0, 500);
                } else {
                    // Two short 100 ms rumbles when turning OFF
                    gamepad1.rumble(0.6, 0.6, 100);
                }
            }


            // ROPE CLIMBING
            boolean holdingOnRope = gamepad2_triangle.toggle(gamepad2.triangle);

            if (holdingOnRope) {
                climbingDirection = 1;  // hold position
            } else if (gamepad2.left_bumper) {
                climbingDirection = 2;  // climb up
            } else if (gamepad2.right_bumper) {
                climbingDirection = -1; // climb down
            } else if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                climbingDirection = 3;
            } else {
                climbingDirection = 0;
            }
            // else: do nothing, keep previous direction (motor holds position)

            // Apply motor control
            if (ropeClimbingAttached) {
                climbRope.ropeClimbing(climbingDirection, -gamepad2.left_stick_y);
            }
            if (gamepad2_dpad_right.pressed(gamepad2.dpad_right) && ropeClimbingAttached) {
                climbRope.rememberHomePosition();
            }

            if (gamepad2_dpad_left.pressed(gamepad2.dpad_left) && ropeClimbingAttached) {
                climbRope.rotateToHome();
            }

            // SPINNING WHEEL
            if (gamepad2_square.toggle((gamepad2.square))) {
                if (isSpinningWheel) {
                    float curTime = (float) runtime.seconds();
                    float elapsedTime = curTime - spinWheelStartTime;

                    telemetry.addData("time spinning wheel: ", elapsedTime);
                    telemetry.addData("over 15 seconds: ", (elapsedTime > 15));

                    if (elapsedTime > 15) {
                        gamepad2.rumble(1, 1, 200);
                    }

                }

                spinWheel.spin(true);
                isSpinningWheel = true;
                spinWheelStartTime = (float) runtime.seconds();
            } else {
                isSpinningWheel = false;
                spinWheel.stop();
            }

            // COLLECTING BALLS
            if (gamepad2.right_trigger > 0) {
                collectingDirection = 1;  // suck in
            } else if (gamepad2.left_trigger > 0) {
                collectingDirection = -1; // let out
            } else {
                collectingDirection = 0;  // hold
            }

            // Apply motor control
            if (collectBallsAttached) {
                collectBalls.collectingBalls(collectingDirection);
            }

            // BALL PUSHER
            if (gamepad2.right_stick_y > 0.3) {
                ballPusher.setMotorStatuses(1);
            } else if (gamepad2.right_stick_y < -0.3) {
                ballPusher.setMotorStatuses(0);
            } else {
                ballPusher.setMotorStatuses(0.5);
            }

            // FLAG RAISING
            boolean needFlagRaised = gamepad2_share.toggle(gamepad2.share);
            if (needFlagRaised && !isFlagRaised && raiseFlagAttached) {
                raiseFlag.setPos(1);
                isFlagRaised = true;
            } else if (!needFlagRaised && isFlagRaised && raiseFlagAttached) {
                raiseFlag.setPos(0);
                isFlagRaised = false;
            }

            //poseestimator
            if (gamepad1.circle) {
                poseEstimator.resetPoseEstimate();
            }


            /* ======================
               Encoder delta -> distance -> pose update
               - read encoders, compute deltas, convert to meters, update estimator
               - ordering: back-left, back-right, front-right, front-left
               ====================== */
            int[] curDriveMotorPositions = driveBase.getEncoderPositions();

            int deltaBL_ticks = curDriveMotorPositions[0] - lastDriveMotorPositions[0];
            int deltaBR_ticks = curDriveMotorPositions[1] - lastDriveMotorPositions[1];
            int deltaFR_ticks = curDriveMotorPositions[2] - lastDriveMotorPositions[2];
            int deltaFL_ticks = curDriveMotorPositions[3] - lastDriveMotorPositions[3];

            // save current positions for next loop
            lastDriveMotorPositions = curDriveMotorPositions.clone();

            double dBL = ticksToDistance(deltaBL_ticks);
            double dBR = ticksToDistance(deltaBR_ticks);
            double dFR = ticksToDistance(deltaFR_ticks);
            double dFL = ticksToDistance(deltaFL_ticks);

            // order: back-left, back-right, front-right, front-left as you requested
            poseEstimator.update(dBL, dBR, dFR, dFL);


            telemetry.addData("cur pose", poseEstimator.getPoseEstimateString());


            //autonomous
            autonomous.showAprilTagData();

            if (gamepad1_options.pressed(gamepad1.options)) {
                gamepad1.rumble(1, 1, 1000);
            }

            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("Heading", imuAngle * 180 / 3.14159265358979323);


            /* ======================
               Drive gears: read bumpers to increment/decrement gear
               - clamps gear between 1 and 3 and maps to DriveGear enum
               ====================== */
            DriveGear currentDriveGear = DriveGear.LOW;

            if (gamepad1_left_bumper.released(gamepad1.left_bumper) && gear >= 2) {
                gear -= 1;
            } else if (gamepad1_right_bumper.released(gamepad1.right_bumper) && gear <= 2) {
                gear += 1;
            }
            telemetry.addData("Gear", gear);

            if (gear == 1) {
                currentDriveGear = DriveGear.LOW;
            } else if (gear == 2) {
                currentDriveGear = DriveGear.MEDIUM;
            } else if (gear == 3) {
                currentDriveGear = DriveGear.HIGH;
            }

            telemetry.addData("drive",drive);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            driveBase.drive (
                    imuAngle, imuPitch,
                    strafe, drive, turn,
                    fieldCentric, currentDriveGear
            );

            telemetry.update();
        } // This brace correctly closes the `while (opModeIsActive())` loop.
    } // This brace correctly closes the `runOpMode()` method.

    /* ======================
       Helper utilities
       ====================== */

    // helper to convert encoder ticks -> linear distance (meters) ----
    private double ticksToDistance(int ticks) {
        return ticks * (WHEEL_CIRCUMFERENCE / TICKS_PER_REV);
    }

} // This brace correctly closes the `EstoniaPanama` class.
