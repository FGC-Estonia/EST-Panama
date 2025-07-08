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
@TeleOp(name = "Main code Estonia Panama")
// allows to display the code in the driver station, comment out to remove
public class EstoniaPanama extends LinearOpMode { //file name is EstoniaPanamas.java    extends the prebuilt LinearOpMode by rev to run
    private double lastFrontBack = 0;
    private double lastLeftRight = 0;
    private double lastTurn = 0;
    private static final double DEADBAND = 0.05;
    private double lastTimeCalledDrive = System.nanoTime();
    private final double SLEW_STEP_FORWARD = 0.05;
    private final double SLEW_STEP_LEFT_RIGHT = 0.05;
    private final double SLEW_STEP_TURN = 0.05;

    //ignore, when joystick has moved very slightly
    private double applyDeadband(double input, double deadband) {
        if (Math.abs(input) < deadband) return 0.0;
        return Math.copySign((Math.abs(input) - deadband) / (1.0 - deadband), input);
    }

    private double cubicScaling(double input) {
        return Math.pow(input, 3);
    }

    /* Limits acceleration changes by at most slewStep, taking into consideration update delays
     * smoothing abrupt inputs into gradual, predictable motion.*/
    private double applySlewRate(double current, double target, double slewStep, double deltaTime) {
        double delta = target - current;
        // maximum change allowed this frame
        double maxStep = slewStep * deltaTime;

        if (Math.abs(delta) > maxStep) {
            delta = Math.signum(delta) * maxStep;
        }
        return current + delta;
    }

    private double clamp(double current, double min, double max) {
        return Math.max(min, Math.min(current, max));
    }

    @Override
    public void runOpMode() {
        boolean protect = true; // activate try/catch to protect the code
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
        MoveRobot moveRobot = new MoveRobot(protect, hardwareMap, telemetry, false);
        MoveRobotTank moveRobotTank = new MoveRobotTank(protect, hardwareMap, telemetry, false);


        Presses gamepad1_left_trigger = new Presses();
        /* Unused controls, which may be put to use in the future.
        Presses gamepad1_right_trigger = new Presses();
        Presses gamepad1_left_bumper = new Presses();
         */
        Presses gamepad1_right_bumper = new Presses();

        // Used to be the HeightToggleGroup, currently kept as a comment for future possible use.
        //Presses.ToggleGroup TempToggleGroup = new Presses.ToggleGroup();
        //Presses gamepad2_cross = new Presses(TempToggleGroup);
        //Presses gamepad2_triangle = new Presses(TempToggleGroup);
        //Presses gamepad2_square = new Presses(TempToggleGroup);
        //Presses gamepad2_circle = new Presses(TempToggleGroup);

        // Used to be the dpad controls, currently not in use but kept as a comment for future convenience.
        //Presses gamepad2_dpad_left = new Presses();
        //Presses gamepad2_dpad_right = new Presses();

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
                //double leftRight = gamepad1.right_stick_x;  //used for tank drive
                double rawLeftRight = gamepad1.left_stick_x;
                double rawFrontBack = -gamepad1.left_stick_y;
                double rawTurn = gamepad1.right_stick_x;
                boolean fieldCentric = gamepad1_left_trigger.toggle(gamepad1.left_trigger > 0.5);
                if (gamepad1_left_trigger.pressed(gamepad1.left_trigger > 0.5)) {
                    gamepad1.rumble(1, 1, 250);
                }
                if (gamepad1_right_bumper.pressed(gamepad1.right_bumper)) {
                    gamepad1.rumble(1, 1, 1000);
                }

                telemetry.addData("Field Centric", fieldCentric);
                telemetry.addData("Heading", imuAngle * 180 / 3.14159265358979323);
                boolean speed1 = gamepad1_cross.toggle(gamepad1.cross);
                boolean speed2 = gamepad1_square.toggle(gamepad1.square);
                boolean speed3 = gamepad1_triangle.toggle(gamepad1.triangle);

                //input processing
                //deadband
                double f_db = applyDeadband(rawFrontBack,  DEADBAND);
                double lr_db = applyDeadband(rawLeftRight,  DEADBAND);
                double t_db = applyDeadband(rawTurn,     DEADBAND);

                //Cubic scaling
                double f_cu = cubicScaling(f_db);
                double lr_cu = cubicScaling(lr_db);
                double t_cu = cubicScaling(t_db);

                //slew
                double now = System.nanoTime();
                double deltaTime = (now - lastTimeCalledDrive) / 1_000_000_000.0; // to seconds
                lastTimeCalledDrive = now;

                double f_slew = applySlewRate(lastFrontBack, f_cu, SLEW_STEP_FORWARD, deltaTime);
                double lr_slew = applySlewRate(lastLeftRight, lr_cu, SLEW_STEP_LEFT_RIGHT, deltaTime);
                double t_slew = applySlewRate(lastTurn, t_cu, SLEW_STEP_TURN, deltaTime);

                lastFrontBack = clamp(f_slew, -1, 1);;
                lastLeftRight = clamp(lr_slew, -1, 1);;
                lastTurn = clamp(t_slew, -1, 1);

                /*moveRobotTank.drive(
                        imuAngle,
                        lastFrontBack, lastLeftRight,
                        speed1, speed2, speed3
                );*/

                moveRobot.move(
                        imuAngle,
                        rawFrontBack, rawLeftRight, lastTurn,
                        fieldCentric,
                        speed1, speed2, speed3
                );

                telemetry.addData("input turn: ", rawTurn);
                telemetry.addData("processedTurn: ", lastTurn);
                telemetry.update();
            } // This brace correctly closes the `while (opModeIsActive())` loop.

        }
    } // This brace correctly closes the `runOpMode()` method.
// This brace correctly closes the `EstoniaPanama` class.