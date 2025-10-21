package org.firstinspires.ftc.teamcode.common.util;

import org.firstinspires.ftc.teamcode.mainModules.MoveRobot.DriveGear;

public interface DriveBaseController {
    void drive(double imuAngle, double imuPitch,
               double drive, double strafe, double turn,
               boolean fieldCentric, DriveGear gear);

    int[] getEncoderPositions();
}
