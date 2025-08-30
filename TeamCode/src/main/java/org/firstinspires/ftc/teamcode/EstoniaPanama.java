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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.util.ImuManager;
import org.firstinspires.ftc.teamcode.mainModules.MoveRobot;
import org.firstinspires.ftc.teamcode.common.util.Presses;
import org.firstinspires.ftc.teamcode.mainModules.MoveRobotTank;
import org.firstinspires.ftc.teamcode.mainModules.ClimbRope;
import org.firstinspires.ftc.teamcode.mainModules.CollectBalls;
import org.firstinspires.ftc.teamcode.mainModules.PoseEstimator;
import org.firstinspires.ftc.teamcode.mainModules.BallPusher;
import org.firstinspires.ftc.teamcode.mainModules.SpinWheel;
import org.firstinspires.ftc.teamcode.mainModules.RaiseFlag;

import org.firstinspires.ftc.teamcode.common.util.DriveBaseController;

import static org.firstinspires.ftc.teamcode.mainModules.MoveRobotTank.DriveGear;

@TeleOp(name = "Main code Estonia Panama")
// allows to display the code in the driver station, comment out to remove
public class EstoniaPanama extends LinearOpMode { //file name is EstoniaPanamas.java    extends the prebuilt LinearOpMode by rev to run
    int climbingDirection = 0; // 0 - stop, 1 - stay on rope, 2 - up, -1 - down, 3 - joystick
    int collectingDirection = 0; // 0 - stop, 1 - in, -1 - out

    // ---- CHANGED: lastDriveMotorPositions length 4 for BL,BR,FR,FL ----
    int[] lastDriveMotorPositions = {0, 0, 0, 0};
    boolean openedBallPusher = false;


    boolean isSpinningWheel = false;
    ElapsedTime runtime = new ElapsedTime();
    float spinWheelStartTime = (float) runtime.seconds();

    // ---- CHANGED: encoder & geometry placeholders (REPLACE with your values) ----
    // TICKS_PER_REV: encoder ticks per motor revolution
    private static final double TICKS_PER_REV = 560.0;      // <-- replace with your encoder
    private static final double WHEEL_DIAMETER = 0.09;;    // meters, replace with your wheel diameter
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    // robot geometry for kinematics: half distances (meters) - replace with your robot measurements
    private static final double HALF_WHEELBASE = 0.155;     // half distance front-back (m) - replace
    private static final double HALF_TRACK = 0.2;         // half distance left-right (m) - replace

    boolean fieldCentric = false;
    DriveBaseController driveBase;
    boolean isFlagRaised = false;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean protect = true; // activate try/catch to protect the code

        driveBase = new MoveRobot(protect, hardwareMap, telemetry, true);

        int storedHomePositionTicks = 0;
        ClimbRope climbRope = null;
        boolean ropeClimbingAttached = false; // leave at false, it detects automatically
        RaiseFlag raiseFlag = null;
        boolean raiseFlagAttached = false;
        CollectBalls collectBalls = null;
        boolean collectBallsAttached = false; // leave at false, it detects automatically
        /*
         * map objects
         * objectName = new ClassName()
         * eg:
         * runMotor = new RunMotor();
         *
         * if te external classes require initialisation do it here
         * eg:
         * RunMotor runMotor = new RunMotor(hardwareMap, telemetry);
         */

        ImuManager imuManager = new ImuManager(protect, hardwareMap, telemetry, true);

        // ---- CHANGED: Construct PoseEstimator with geometry ----
        PoseEstimator poseEstimator = new PoseEstimator(imuManager, HALF_WHEELBASE, HALF_TRACK);

        // ---- NOTE: moveRobot is created here and we'll use its getEncoderPos() below ----
       // MoveRobot moveRobot = new MoveRobot(protect, hardwareMap, telemetry, true);
        //MoveRobotTank moveRobotTank = new MoveRobotTank(protect, hardwareMap, telemetry, true);
        BallPusher ballPusher = new BallPusher(hardwareMap, telemetry);
        SpinWheel spinWheel = new SpinWheel(hardwareMap, telemetry);
        //Autonomous autonomous = new Autonomous(moveRobot, hardwareMap,telemetry);

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


        Presses gamepad1_left_trigger = new Presses();
        // Unused controls, which may be put to use in the future.
        Presses gamepad1_right_trigger = new Presses();

        //for rope climbing
        Presses gamepad2_cross = new Presses();
        Presses gamepad2_triangle = new Presses();

        //for spinning wheel
        Presses gamepad2_square = new Presses();


        Presses gamepad1_right_bumper = new Presses();
        Presses gamepad1_left_bumper = new Presses();

        // Used to be the HeightToggleGroup, currently kept as a comment for future possible use.
        //Presses.ToggleGroup TempToggleGroup = new Presses.ToggleGroup();
        //Presses gamepad2_cross = new Presses(TempToggleGroup);
        //Presses gamepad2_triangle = new Presses(TempToggleGroup);
        //Presses gamepad2_square = new Presses(TempToggleGroup);
        //Presses gamepad2_circle = new Presses(TempToggleGroup);

        Presses gamepad1_dpad_left = new Presses();
        Presses gamepad1_dpad_right = new Presses();
        Presses gamepad1_dpad_up = new Presses();
        Presses gamepad1_dpad_down = new Presses();
        Presses gamepad2_dpad_left = new Presses();
        Presses gamepad2_dpad_right = new Presses();
        Presses gamepad2_dpad_up = new Presses();
        Presses gamepad2_dpad_down = new Presses();
        Presses gamepad2_left_bumper = new Presses();

        // For FieldCentric toggle and GyroReset
        Presses gamepad1_share = new Presses();
        Presses gamepad1_options = new Presses();

        Presses gamepad2_share = new Presses();

        Presses.ToggleGroup speedSelectToggle = new Presses.ToggleGroup();
        Presses gamepad1_square = new Presses(speedSelectToggle);
        Presses gamepad1_triangle = new Presses(speedSelectToggle);
        Presses gamepad1_circle = new Presses(speedSelectToggle);
        Presses gamepad1_cross = new Presses(speedSelectToggle);
        gamepad1_triangle.setToggleTrue();//set default value

        int gear = 1;

        // double maxRaisedVelocity; // This variable is declared but unused, and causes a warning. Removed for cleaner code unless it has a future use.

        telemetry.update();
        waitForStart(); //everything has been initialized, waiting for the start button
        while (opModeIsActive()) { // main loop

            //gyro reset
            if (gamepad1.options) {
                imuManager.resetImu();
            }

            //move robot
            //position automatically when pressed

            double imuAngle = imuManager.getYawRadians();
            double imuPitch = imuManager.getPitchRadians();
            //double drive = gamepad1.right_stick_x;  //used for tank drive
            double drive = -gamepad1.left_stick_x;
            double strafe = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            boolean holdingOnRope = gamepad2_triangle.toggle(gamepad2.triangle);
            int climbingDirection = 0;

            // FieldCentric rumble
            if (gamepad1_share.pressed(gamepad1.share)) {
                fieldCentric = !fieldCentric;
                //telemetry.addData("pressed share", "");
                if (fieldCentric) {
                    // One long 500 ms rumble when turning ON
                    gamepad1.rumble(1.0, 1.0, 500);
                } else {
                    // Two short 100 ms rumbles when turning OFF
                    gamepad1.rumble(0.6, 0.6, 100);
                }
            }


            // Climbing logic with dpad
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

            //Spinning wheel

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

            //Collecting balls
            if (gamepad2.right_trigger > 0) {
                collectingDirection = 1;  // suck in
            } else if (gamepad2.left_trigger > 0) {
                collectingDirection = -1; // let out
            } else {
                collectingDirection = 0;  // hold
            }



            /*
            telemetry.addData("opened ball pusher", openedBallPusher);
            telemetry.addData("need open ball pusher", needToOpenBallPusher);
            telemetry.addData("holdiong cross", gamepad2.cross);*/


            // Apply motor control
            if (collectBallsAttached) {
                collectBalls.collectingBalls(collectingDirection);
            }

            //poseestimator
            if (gamepad1.circle) {
                poseEstimator.resetPoseEstimate();
            }

            //ball pusher
            if (gamepad2.right_stick_y > 0.3) {
                ballPusher.setMotorStatuses(1);
            } else if (gamepad2.right_stick_y < -0.3) {
                ballPusher.setMotorStatuses(0);
            } else {
                ballPusher.setMotorStatuses(0.5);
            }


            /*boolean needToOpenBallPusher = gamepad2_dpad_right.toggle(gamepad2.dpad_right);

            if (needToOpenBallPusher && !openedBallPusher) {
                openedBallPusher = true;
                ballPusher.open(1400);
            } else if (!needToOpenBallPusher && openedBallPusher) {
                openedBallPusher = false;
                ballPusher.close(1600);
            }
            */

            //flag
            boolean needFlagRaised = gamepad2_share.toggle(gamepad2.share);
            if (needFlagRaised && !isFlagRaised && raiseFlagAttached) {
                raiseFlag.setPos(1);
                isFlagRaised = true;
            } else if (!needFlagRaised && isFlagRaised && raiseFlagAttached) {
                raiseFlag.setPos(0);
                isFlagRaised = false;
            }


            // ---- CHANGED: read 4 encoder positions via driveBase ----
// Expecting order: {back-left, back-right, front-right, front-left}
            int[] curDriveMotorPositions = driveBase.getEncoderPositions();

// ---- CHANGED: compute tick deltas per wheel ----
            int deltaBL_ticks = curDriveMotorPositions[0] - lastDriveMotorPositions[0];
            int deltaBR_ticks = curDriveMotorPositions[1] - lastDriveMotorPositions[1];
            int deltaFR_ticks = curDriveMotorPositions[2] - lastDriveMotorPositions[2];
            int deltaFL_ticks = curDriveMotorPositions[3] - lastDriveMotorPositions[3];

            // save current positions for next loop
            lastDriveMotorPositions = curDriveMotorPositions.clone();

            // ---- CHANGED: convert tick deltas -> linear displacements and call PoseEstimator.update ----
            double dBL = ticksToDistance(deltaBL_ticks);
            double dBR = ticksToDistance(deltaBR_ticks);
            double dFR = ticksToDistance(deltaFR_ticks);
            double dFL = ticksToDistance(deltaFL_ticks);

            // order: back-left, back-right, front-right, front-left as you requested
            poseEstimator.update(dBL, dBR, dFR, dFL);


            telemetry.addData("cur pose", poseEstimator.getPoseEstimateString());


            //autonomous
            //autonomous.showAprilTagData();

            if (gamepad1_options.pressed(gamepad1.options)) {
                gamepad1.rumble(1, 1, 1000);
            }

            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("Heading", imuAngle * 180 / 3.14159265358979323);
            //telemetry.addData("Pitch", imuPitch * 180 / 3.14159265358979323);

            DriveGear currentDriveGear = DriveGear.LOW;

            if (gamepad1_left_bumper.released(gamepad1.left_bumper) && gear >= 2) {
                gear -= 1;
            } else if (gamepad1_right_bumper.released(gamepad1.right_bumper) && gear <= 2) {
                gear += 1;
            }
            //telemetry.addData("Gear", gear);

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
            driveBase.drive(
                    imuAngle, imuPitch,
                    strafe, drive, turn,
                    fieldCentric, currentDriveGear
            );

            telemetry.update();
        } // This brace correctly closes the `while (opModeIsActive())` loop.
    } // This brace correctly closes the `runOpMode()` method.

    // ---- CHANGED: helper to convert encoder ticks -> linear distance (meters) ----
    private double ticksToDistance(int ticks) {
        return ticks * (WHEEL_CIRCUMFERENCE / TICKS_PER_REV);
    }

} // This brace correctly closes the `EstoniaPanama` class.
