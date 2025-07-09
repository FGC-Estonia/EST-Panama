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
import org.firstinspires.ftc.teamcode.mainModules.ImuManager;
import org.firstinspires.ftc.teamcode.mainModules.MoveRobot;
import org.firstinspires.ftc.teamcode.mainModules.Presses;
import org.firstinspires.ftc.teamcode.mainModules.MoveRobotTank;
import org.firstinspires.ftc.teamcode.mainModules.ClimbRope;
@TeleOp(name = "Main code Estonia Panama")
// allows to display the code in the driver station, comment out to remove
public class EstoniaPanama extends LinearOpMode { //file name is EstoniaPanamas.java    extends the prebuilt LinearOpMode by rev to run
    int climbingDirection = 0; // 0 - stop, 1 - up, -1 - down

    @Override
    public void runOpMode() {
        boolean protect = true; // activate try/catch to protect the code
        boolean xDrive = true;
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

        ImuManager imuManager = new ImuManager(protect, hardwareMap, telemetry);
        MoveRobot moveRobot = new MoveRobot(protect, hardwareMap, telemetry, true);
        MoveRobotTank moveRobotTank = new MoveRobotTank(protect, hardwareMap, telemetry, true);
        ClimbRope climbRope = new ClimbRope(protect, hardwareMap, telemetry);


        Presses gamepad1_left_trigger = new Presses();
        // Unused controls, which may be put to use in the future.
        Presses gamepad1_right_trigger = new Presses();

        //for rope climbing
        Presses gamepad2_cross = new Presses();

        Presses gamepad1_right_bumper = new Presses();

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


        Presses.ToggleGroup speedSelectToggle = new Presses.ToggleGroup();
        Presses gamepad1_square = new Presses(speedSelectToggle);
        Presses gamepad1_triangle = new Presses(speedSelectToggle);
        Presses gamepad1_circle = new Presses(speedSelectToggle);
        Presses gamepad1_cross = new Presses(speedSelectToggle);
        gamepad1_triangle.setToggleTrue();//set default value



        // double maxRaisedVelocity; // This variable is declared but unused, and causes a warning. Removed for cleaner code unless it has a future use.

        telemetry.update();
        waitForStart(); //everything has been initialized, waiting for the start button

        while (opModeIsActive()) { // main loop

            //gyro reset
            if (gamepad1.right_bumper) {
                imuManager.resetImu();
            }

            //move robot
                //position automatically when pressed


                double imuAngle = imuManager.getYawRadians();
                double imuAngle = imuManager.getYawRadians();
                double imuPitch = imuManager.getPitchRadians();
                //double leftRight = gamepad1.right_stick_x;  //used for tank drive
                double leftRight = gamepad1.left_stick_x;
                double frontBack = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                boolean fieldCentric = gamepad1_left_trigger.toggle(gamepad1.left_trigger > 0.5);
                int climbingDirection = 0;
                int previousClimbDirection = 0;

            // Climb mode state: 1 = up, -1 = down, 0 = stopped
                if (gamepad1_dpad_up.pressed(gamepad1.dpad_up)) {
                    climbingDirection = 1;
                }
                if (gamepad1_dpad_down.pressed(gamepad1.dpad_down)) {
                    climbingDirection = -1;
                }

// Pause toggle
            if (gamepad1_dpad_left.pressed(gamepad1.dpad_left)) {
                climbingDirection = (climbingDirection == 0) ? previousClimbDirection : 0;
            }

            if (climbingDirection != 0) {
                previousClimbDirection = climbingDirection;
                gamepad1.rumble(1, 1, 200);
            }

// Apply motor control
            climbRope.ropeClimbing(climbingDirection);




            if (gamepad1_right_bumper.pressed(gamepad1.right_bumper)) {
                    gamepad1.rumble(1, 1, 1000);
                }

                telemetry.addData("Field Centric", fieldCentric);
                telemetry.addData("Heading", imuAngle * 180 / 3.14159265358979323);
                telemetry.addData("Pitch", imuPitch * 180 / 3.14159265358979323);
                boolean speed1 = gamepad1_cross.toggle(gamepad1.cross);
                boolean speed2 = gamepad1_square.toggle(gamepad1.square);
                boolean speed3 = gamepad1_triangle.toggle(gamepad1.triangle);

                climbRope.ropeClimbing(climbingDirection);

                if (!xDrive) {
                    moveRobotTank.drive(
                            imuAngle, imuPitch,
                            frontBack, turn,
                            speed1, speed2, speed3
                    );
                } else {
                    moveRobot.move(
                            imuAngle,
                            frontBack, leftRight, turn,
                            fieldCentric,
                            speed1, speed2, speed3
                    );
                }
                telemetry.update();
            } // This brace correctly closes the `while (opModeIsActive())` loop.

        }
    } // This brace correctly closes the `runOpMode()` method.
// This brace correctly closes the `EstoniaPanama` class.