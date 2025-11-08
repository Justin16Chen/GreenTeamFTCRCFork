package org.firstinspires.ftc.teamcode.utils.misc;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Field {
    public static double tileTeethLength = 0.8; // length of teeth of field tiles (in inches)
    public static double goalY = 66, blueGoalX = -69, redGoalX = 69;
    public static double blueParkX = 24 + 0.4 + 9, redParkX = -24 - 0.4 - 9, parkY = -48 + 0.4 + 9, parkADeg = 90;
    public static double blueResetX = 72 - Robot.width * 0.5, redResetX = -72 + Robot.width * 0.5, resetY = -72 + Robot.frontToCenterLength, blueResetADeg = 0, redResetADeg = 180;
}
